"""График кадра 0x03: 20 отсчётов ЭМГ, угол и метка.

По BLE программа только слушает notify FFF1 и сама ничего не пишет.
START (кадр 04 01 в FFF2) нужно послать отдельно, иначе измерения не начнутся.
По COM читается тот же поток байтов, что плата отдала бы в notify.
"""

import argparse
import asyncio
import sys
import threading
import time
from collections import deque

from frame_parser import SAMPLE_TYPE, FrameParser

POWER_TYPE = 0x01
EVENT_TYPE = 0x02
CHARGE_TEXT = {
    0: "кабель снят",
    1: "заряд идёт",
    2: "кабель есть, заряд окончен",
}
EVENT_TEXT = {
    1: "кабель вставлен, измерения запрещены",
    2: "остановка из-за низкого заряда",
    3: "измерение остановлено",
}
ERROR_TEXT = {
    0: "ошибки нет",
    1: "BNO055: угол неверен",
    2: "BNO055 не ответил по I2C",
}
# Окно кадра 0x03. Метка в кадре снята с сетки 5 мс, у соседних кадров она отличается на 10.
IDEAL_GAP_MS = 10
# Дырка: окно 10 мс не попало в очередь, следующая метка ушла ещё на один кадр.
GAP_DROP_MS = 20
# AT+CONPARAMS=6: интервал связи 6 × 1,25 мс. Уведомления телефона режутся этим шагом.
BLE_CONN_MS = 7.5
# Паузы, которые прошивка ждёт всегда до рекламы: MOD 200+200, выход BNO 650,
# на каждый из двух датчиков 19 мс, 7 мс и хотя бы один опрос кватерниона 10 мс.
# Ответы AT и сброс модуля идут сверху. В кадр 0x03 это время не записывается.
IDEAL_BOOT_FLOOR_MS = 400 + 650 + 2 * (19 + 7 + 10)


def describe_power(frame):
    """Последний кадр 0x01: милливольты и байт зарядки."""
    if frame is None:
        return "Зарядка: ещё не приходила"
    code = frame["charge"]
    meaning = CHARGE_TEXT.get(code, "неизвестный код")
    return f"Зарядка: {frame['voltage_mV']} мВ, {meaning} ({code})"


def describe_event(frame):
    """Последний кадр 0x02: событие и, для остановки, код ошибки."""
    if frame is None:
        return "Устройство: событий не было"
    event = frame["event"]
    code = frame["code"]
    what = EVENT_TEXT.get(event, "неизвестное событие")
    if event == 3:
        err = ERROR_TEXT.get(code, "неизвестный код ошибки")
        return f"Устройство: {what}, {err} ({event}/{code})"
    return f"Устройство: {what} ({event})"


def describe_gap(timing, over_ble):
    """Интервал соседних 0x03: по метке платы и по часам компьютера."""
    if timing["gap_n"] == 0:
        stamp = "по метке ещё нет двух кадров"
    else:
        stamp = (
            f"по метке {timing['gap_ms']} мс, "
            f"разброс {timing['gap_min']}…{timing['gap_max']}, "
            f"дырки ≥{GAP_DROP_MS} мс: {timing['gap_drop']} из {timing['gap_n']}"
        )
    if timing["arrival_n"] == 0:
        arrived = "приход в программу ещё не с чем сравнить"
    else:
        mean = timing["arrival_sum"] / timing["arrival_n"]
        arrived = f"приход сюда {timing['arrival_ms']:.0f} мс, среднее {mean:.0f}"
    if over_ble:
        arrival_ideal = (
            f"идеал прихода около {IDEAL_GAP_MS} мс, "
            f"уведомления идут раз в {BLE_CONN_MS:g} мс и несколько кадров бывают в одной пачке"
        )
    else:
        arrival_ideal = f"идеал прихода {IDEAL_GAP_MS} мс"
    return (
        f"Между кадрами: {stamp}. Идеал метки {IDEAL_GAP_MS} мс "
        f"(15 мс — следующий шаг сетки).\n"
        f"{arrived[0].upper()}{arrived[1:]}. {arrival_ideal[0].upper()}{arrival_ideal[1:]}."
    )


def describe_ready(timing):
    """Метка первого кадра. Сетка стартует после инициализации, не вместе с питанием."""
    if timing["first_stamp"] is None:
        measured = "кадра 0x03 ещё не было"
    else:
        measured = f"метка первого кадра {timing['first_stamp']} мс от старта сетки"
    return (
        f"До кадра: {measured}. От START до кадра идеал {IDEAL_GAP_MS} мс, на сетке бывает 15.\n"
        f"До рекламы паузы прошивки не короче {IDEAL_BOOT_FLOOR_MS} мс плюс ответы AT. "
        f"В метку это не входит."
    )


def sample_points(frame):
    """Время отсчёта: метка кадра — конец окна, шаг внутри окна 0,5 мс."""
    end = frame["time_ms"] / 1000.0
    count = len(frame["emg"])
    step = 0.0005
    start = end - (count - 1) * step
    return [(start + index * step, value) for index, value in enumerate(frame["emg"])], (
        end,
        frame["angle_x10"] / 10.0,
    )


class Live:
    def __init__(self, window):
        self.window = window
        self.lock = threading.Lock()
        self.parser = FrameParser()
        self.emg = deque()
        self.angle = deque()
        self.samples = 0
        self.frames = 0
        self.power = None
        self.event = None
        self.message = "Ожидание кадра 0x03"
        self.prev_stamp = None
        self.gap_ms = None
        self.gap_min = None
        self.gap_max = None
        self.gap_n = 0
        self.gap_drop = 0
        self.first_stamp = None
        self.prev_arrival = None
        self.arrival_ms = None
        self.arrival_sum = 0.0
        self.arrival_n = 0

    def note_sample(self, frame, arrived):
        """Метку и приход считает только кадр 0x03. Питание и события интервал не двигают."""
        stamp = frame["time_ms"]
        if self.first_stamp is None:
            self.first_stamp = stamp
        if self.prev_stamp is not None:
            gap = stamp - self.prev_stamp
            self.gap_ms = gap
            self.gap_n += 1
            if self.gap_min is None or gap < self.gap_min:
                self.gap_min = gap
            if self.gap_max is None or gap > self.gap_max:
                self.gap_max = gap
            if gap >= GAP_DROP_MS:
                self.gap_drop += 1
        self.prev_stamp = stamp
        if self.prev_arrival is not None:
            self.arrival_ms = (arrived - self.prev_arrival) * 1000.0
            self.arrival_sum += self.arrival_ms
            self.arrival_n += 1
        self.prev_arrival = arrived

    def timing(self):
        return {
            "gap_ms": self.gap_ms,
            "gap_min": self.gap_min,
            "gap_max": self.gap_max,
            "gap_n": self.gap_n,
            "gap_drop": self.gap_drop,
            "first_stamp": self.first_stamp,
            "arrival_ms": self.arrival_ms,
            "arrival_sum": self.arrival_sum,
            "arrival_n": self.arrival_n,
        }

    def add(self, chunk):
        arrived = time.perf_counter()
        frames = self.parser.feed(chunk)
        with self.lock:
            for frame in frames:
                self.frames += 1
                if frame["type"] == POWER_TYPE:
                    self.power = frame
                    continue
                if frame["type"] == EVENT_TYPE:
                    self.event = frame
                    continue
                if frame["type"] != SAMPLE_TYPE:
                    continue
                self.note_sample(frame, arrived)
                points, angle = sample_points(frame)
                self.emg.extend(points)
                self.angle.append(angle)
                self.samples += len(points)
            if self.emg:
                newest = self.emg[-1][0]
                while self.emg and newest - self.emg[0][0] > self.window:
                    self.emg.popleft()
                while self.angle and newest - self.angle[0][0] > self.window:
                    self.angle.popleft()

    def snapshot(self):
        with self.lock:
            return (
                list(self.emg),
                list(self.angle),
                self.samples,
                self.frames,
                self.message,
                self.power,
                self.event,
                self.timing(),
            )


def run_com(args, live, stop):
    import serial

    port = serial.Serial(args.port, args.baud, timeout=0.05)
    live.message = f"{args.port}: слушаю кадры"
    try:
        while not stop.is_set():
            chunk = port.read(256)
            if chunk:
                live.add(chunk)
    finally:
        port.close()


async def run_ble(args, live, stop):
    import bleak

    live.message = f"Поиск {args.name}"

    def on_notify(_char, data):
        live.add(bytes(data))

    devices = await bleak.BleakScanner.discover(timeout=args.scan)
    chosen = None
    for device in devices:
        if device.name == args.name:
            chosen = device
            break
    if chosen is None:
        live.message = f"{args.name} не найден"
        return
    async with bleak.BleakClient(chosen) as client:
        char = None
        for service in client.services:
            for item in service.characteristics:
                if args.char.lower() in item.uuid.lower() and "notify" in item.properties:
                    char = item
                    break
        if char is None:
            live.message = "Notify FFF1 не найден"
            return
        await client.start_notify(char, on_notify)
        live.message = "BLE: жду кадр 0x03. START эта программа не шлёт."
        while not stop.is_set() and client.is_connected:
            await asyncio.sleep(0.1)
        await client.stop_notify(char)


def ble_thread(args, live, stop):
    try:
        asyncio.run(run_ble(args, live, stop))
    except Exception as error:
        live.message = f"BLE: {error}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ble", action="store_true", help="Слушать notify FFF1")
    parser.add_argument("--port", help="COM-порт с тем же потоком байтов")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--name", default="E104-BT5032A")
    parser.add_argument("--char", default="fff1")
    parser.add_argument("--scan", type=float, default=6.0)
    parser.add_argument("--window", type=float, default=5.0)
    args = parser.parse_args()
    if args.ble == bool(args.port):
        parser.error("Нужен один источник: --ble или --port COMx")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    live = Live(args.window)
    stop = threading.Event()
    if args.ble:
        worker = threading.Thread(target=ble_thread, args=(args, live, stop), daemon=True)
    else:
        worker = threading.Thread(target=run_com, args=(args, live, stop), daemon=True)
    worker.start()

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(9.5, 7.2))
    figure.subplots_adjust(bottom=0.34)
    emg_line, = axes[0].plot([], [], linewidth=0.8)
    angle_line, = axes[1].plot([], [], linewidth=1.0)
    axes[0].set_ylabel("ЭМГ, код АЦП")
    axes[1].set_ylabel("Угол, градусы")
    axes[1].set_xlabel("Время кадра, с")
    axes[1].set_ylim(0, 180)
    notice = figure.text(0.01, 0.01, "", va="bottom", fontsize=8)

    def update(_frame):
        emg, angle, samples, frames, message, power, event, timing = live.snapshot()
        if emg:
            xs, ys = zip(*emg)
            emg_line.set_data(xs, ys)
            axes[0].set_xlim(xs[0], xs[-1] if xs[-1] > xs[0] else xs[0] + 0.01)
            low, high = min(ys), max(ys)
            pad = max(1, (high - low) * 0.1)
            axes[0].set_ylim(low - pad, high + pad)
        if angle:
            xs, ys = zip(*angle)
            angle_line.set_data(xs, ys)
        notice.set_text(
            f"{message} | кадров: {frames} | отсчётов ЭМГ: {samples}\n"
            f"{describe_gap(timing, args.ble)}\n"
            f"{describe_ready(timing)}\n"
            f"{describe_power(power)}\n"
            f"{describe_event(event)}"
        )
        return emg_line, angle_line, notice

    animation = FuncAnimation(figure, update, interval=40, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        del animation


if __name__ == "__main__":
    sys.exit(main())
