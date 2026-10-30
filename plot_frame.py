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
        self.message = "Ожидание кадра 0x03"

    def add(self, chunk):
        frames = self.parser.feed(chunk)
        with self.lock:
            for frame in frames:
                self.frames += 1
                if frame["type"] != SAMPLE_TYPE:
                    continue
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
            return list(self.emg), list(self.angle), self.samples, self.frames, self.message


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

    figure, axes = plt.subplots(2, 1, sharex=True)
    emg_line, = axes[0].plot([], [], linewidth=0.8)
    angle_line, = axes[1].plot([], [], linewidth=1.0)
    axes[0].set_ylabel("ЭМГ, код АЦП")
    axes[1].set_ylabel("Угол, градусы")
    axes[1].set_xlabel("Время кадра, с")
    axes[1].set_ylim(0, 180)
    notice = figure.text(0.01, 0.01, "")

    def update(_frame):
        emg, angle, samples, frames, message = live.snapshot()
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
        notice.set_text(f"{message} | кадров: {frames} | отсчётов ЭМГ: {samples}")
        return emg_line, angle_line, notice

    animation = FuncAnimation(figure, update, interval=40, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        del animation


if __name__ == "__main__":
    sys.exit(main())
