"""График кадра 0x03: 20 отсчётов ЭМГ, угол и метка.

COM по умолчанию COM9, 115200, 8N1: RTS выключен, DTR включён, без управления потоком.
BLE: модуль E104-BT5032A, notify FFF1, команды в FFF2.
Измерения начинаются только кнопкой «Старт» (кадр 04 01) и кончаются кнопкой «Стоп» (04 02).
«Подключить» и «Отключить» открывают и закрывают связь. Перед закрытием уходит «Стоп».
"""

import argparse
import asyncio
import math
import queue
import sys
import threading
import time
from collections import deque
from uuid import UUID

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
DEFAULT_NAME = "E104-BT5032A"
DEFAULT_CHARACTERISTIC = "fff1"
DEFAULT_WRITE = "fff2"
DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
START_CMD = bytes((0x04, 0x01))
STOP_CMD = bytes((0x04, 0x02))
# Паузы, которые прошивка ждёт всегда до рекламы: MOD 200+200, выход BNO 650,
# на каждый датчик CONFIG 19, RST_SYS 700, ещё 50, снова CONFIG 19,
# паузы питания и часов по 10 и выдержка NDOF 600. Опрос Эйлера сверху не входит.
# Ответы AT и сброс модуля идут сверху. В кадр 0x03 это время не записывается.
IDEAL_BOOT_FLOOR_MS = 400 + 650 + 2 * (19 + 700 + 50 + 19 + 10 + 10 + 600)


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
        self.commands = queue.Queue()
        self.serial = 0
        self.connected = False
        self.mode = "idle"
        self.cable_event = False
        self.low_battery = False
        self.command_at = None
        self.measurement = "Измерения не запущены"
        self.warning = ""
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

    def set_message(self, message):
        with self.lock:
            self.message = message
        print(message, flush=True)

    def _blocked(self):
        charge = None if self.power is None else self.power["charge"]
        return self.low_battery or self.cable_event or charge not in (None, 0)

    def connection_opened(self):
        with self.lock:
            self.parser.reset()
            self.emg.clear()
            self.angle.clear()
            self.samples = 0
            self.frames = 0
            self.power = None
            self.event = None
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
            self.cable_event = False
            self.low_battery = False
            self.mode = "idle"
            self.measurement = "Измерения не запущены"
            self.warning = ""
            self.command_at = None
            self.serial += 1
            self.connected = True

    def connection_closed(self):
        with self.lock:
            self.connected = False
            self.serial += 1
            self.mode = "disconnected"
            self.measurement = "Связь закрыта; состояние измерений не подтверждено"
            self.command_at = None
            self.parser.reset()

    def request(self, command):
        if command not in ("start", "stop"):
            raise ValueError("Неизвестная команда")
        with self.lock:
            if not self.connected:
                self.warning = "Сначала подключитесь к устройству"
                return False
            if command == "start" and (self._blocked() or self.mode in ("measuring", "start_pending", "stop_pending")):
                self.warning = "Старт сейчас недоступен: проверьте кабель, заряд и состояние измерений"
                return False
            if command == "stop" and self.mode == "stop_pending":
                return False
            self.serial += 1
            if command == "start":
                self.warning = ""
            self.command_at = time.monotonic()
            self.mode = "start_pending" if command == "start" else "stop_pending"
            self.measurement = (
                "Старт в очереди; ждём кадры 0x03"
                if command == "start"
                else "Стоп в очереди; ждём событие остановки"
            )
            self.commands.put((self.serial, command))
            return True

    def next_command(self):
        with self.lock:
            while True:
                try:
                    token, command = self.commands.get_nowait()
                except queue.Empty:
                    return None
                if token != self.serial or not self.connected:
                    continue
                if command == "start" and self._blocked():
                    continue
                payload = START_CMD if command == "start" else STOP_CMD
                return token, command, payload

    def command_written(self, token, command):
        with self.lock:
            expected = "start_pending" if command == "start" else "stop_pending"
            if token == self.serial and self.mode == expected:
                self.measurement = (
                    "04 01 передано; отдельного ответа нет, ждём кадры 0x03"
                    if command == "start"
                    else "04 02 передано; ждём событие 02 03"
                )

    def command_failed(self, token, error):
        with self.lock:
            if token == self.serial:
                self.mode = "unknown"
                self.measurement = "Команда не передана; состояние устройства неизвестно"
                self.warning = f"Ошибка записи: {error}"

    def _stopped(self, message):
        self.serial += 1
        self.mode = "idle"
        self.measurement = message
        self.command_at = None

    def _note_timeout(self):
        if self.command_at is None or time.monotonic() - self.command_at <= 3:
            return
        if self.mode == "start_pending":
            self.mode = "start_unconfirmed"
            self.measurement = "Нет подтверждения старта: кадры 0x03 не приходят"
        elif self.mode == "stop_pending":
            self.measurement = "Нет подтверждения стопа: событие 02 03 не пришло"

    def controls(self):
        return {
            "measurement": self.measurement,
            "warning": self.warning,
            "can_start": self.connected and not self._blocked() and self.mode not in (
                "measuring", "start_pending", "stop_pending"),
            "can_stop": self.connected and self.mode != "stop_pending",
        }

    def add(self, chunk):
        arrived = time.perf_counter()
        with self.lock:
            frames = self.parser.feed(chunk)
            for frame in frames:
                self.frames += 1
                if frame["type"] == POWER_TYPE:
                    self.power = frame
                    self._note_power(frame)
                    continue
                if frame["type"] == EVENT_TYPE:
                    self.event = frame
                    self._note_event(frame)
                    continue
                if frame["type"] != SAMPLE_TYPE:
                    continue
                self.note_sample(frame, arrived)
                points, angle = sample_points(frame)
                self.emg.extend(points)
                self.angle.append(angle)
                self.samples += len(points)
                if self.mode != "stop_pending" and not self._blocked():
                    self.mode = "measuring"
                    self.measurement = "Идут измерения"
                elif self._blocked():
                    self.warning = "Пришли измерения при запрете питания"
            if self.emg:
                newest = self.emg[-1][0]
                while self.emg and newest - self.emg[0][0] > self.window:
                    self.emg.popleft()
                while self.angle and newest - self.angle[0][0] > self.window:
                    self.angle.popleft()

    def _note_power(self, frame):
        charge = frame["charge"]
        if charge == 0:
            was_cable = self.cable_event
            self.cable_event = False
            if was_cable and not self.low_battery and self.mode == "idle":
                self.measurement = "Кабель снят: готово к старту"
        elif charge in (1, 2):
            self.cable_event = True
            self._stopped("Кабель подключён: измерения запрещены")
        else:
            self.warning = f"Неизвестный код зарядки: {charge}"

    def _note_event(self, frame):
        event, code = frame["event"], frame["code"]
        if event == 1:
            self.cable_event = True
            self._stopped("Кабель вставлен: измерения запрещены")
        elif event == 2:
            self.low_battery = True
            self._stopped("Принудительная остановка: низкий заряд")
        elif event == 3:
            self._stopped("Измерение остановлено: " + ERROR_TEXT.get(code, f"неизвестная ошибка {code}"))
            if code:
                self.warning = ERROR_TEXT.get(code, f"Неизвестная ошибка {code}")
        else:
            self.warning = f"Неизвестное событие {event}, ошибка {code}"

    def snapshot(self):
        with self.lock:
            self._note_timeout()
            controls = self.controls()
            return (
                list(self.emg),
                list(self.angle),
                self.samples,
                self.frames,
                self.message,
                self.power,
                self.event,
                self.timing(),
                controls,
            )


def normalize_uuid(value):
    value = value.strip().lower().removeprefix("0x")
    if len(value) in (4, 8):
        value = f"{int(value, 16):08x}-0000-1000-8000-00805f9b34fb"
    return str(UUID(value))


def select_device(discovered, name, address=None):
    matches = []
    for device, advertisement in discovered.values():
        advertised_name = advertisement.local_name or device.name or ""
        if address:
            match = device.address.casefold() == address.casefold()
        else:
            match = advertised_name.casefold() == name.casefold()
        if match:
            matches.append(device)
    if not matches:
        raise LookupError(f"Не найдено BLE-устройство {address or name}. Проверьте питание и advertising.")
    if len(matches) != 1:
        addresses = ", ".join(device.address for device in matches)
        raise LookupError(f"Найдено несколько устройств с этим именем: {addresses}. Укажите --address.")
    return matches[0]


def select_characteristic(services, char_uuid, service_uuid=None):
    target = normalize_uuid(char_uuid)
    service_filter = normalize_uuid(service_uuid) if service_uuid else None
    candidates = []
    available = []
    for service in services:
        for characteristic in service.characteristics:
            properties = set(characteristic.properties)
            if properties.intersection(("notify", "indicate")):
                available.append(f"{characteristic.uuid} (service {service.uuid})")
                if (normalize_uuid(characteristic.uuid) == target and
                        (service_filter is None or normalize_uuid(service.uuid) == service_filter)):
                    candidates.append(characteristic)
    if len(candidates) != 1:
        detail = "; ".join(available) or "нет"
        raise LookupError(f"Нужна одна Notify/Indicate-характеристика {target}, найдено {len(candidates)}. "
                          f"Доступные: {detail}. При совпадающих UUID задайте --service-uuid.")
    return candidates[0]


def select_write(services, write_uuid, service_uuid, notify):
    target = normalize_uuid(write_uuid)
    service_filter = normalize_uuid(service_uuid) if service_uuid else None
    candidates = []
    available = []
    for service in services:
        for characteristic in service.characteristics:
            properties = set(characteristic.properties)
            writable = properties.intersection(("write", "write-without-response"))
            if writable:
                available.append(f"{characteristic.uuid} (service {service.uuid})")
            if service_filter is not None and normalize_uuid(service.uuid) != service_filter:
                continue
            if writable and normalize_uuid(characteristic.uuid) == target:
                candidates.append(characteristic)
    if notify is not None:
        same = [
            item for item in candidates
            if normalize_uuid(getattr(item, "service_uuid", "")) == normalize_uuid(notify.service_uuid)
        ]
        if same:
            candidates = same
    if len(candidates) != 1:
        detail = "; ".join(available) or "нет"
        raise LookupError(
            f"Нужна одна Write-характеристика {target}, найдено {len(candidates)}. "
            f"Доступные: {detail}. При совпадающих UUID задайте --service-uuid."
        )
    return candidates[0]


def response_mode(characteristic, write_mode):
    properties = set(characteristic.properties)
    use_response = "write" in properties if write_mode == "auto" else write_mode == "response"
    required = "write" if use_response else "write-without-response"
    if required not in properties:
        raise ValueError(f"FFF2 не поддерживает {required}")
    return use_response


class OperationInterrupted(Exception):
    pass


async def interruptible(awaitable, stop, enabled, timeout):
    task = asyncio.ensure_future(awaitable)
    deadline = time.monotonic() + timeout
    try:
        while not task.done():
            if stop.is_set() or not enabled.is_set():
                raise OperationInterrupted()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"BLE-операция не завершилась за {timeout:g} с")
            await asyncio.wait({task}, timeout=0.10)
        return await task
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


async def send_command(client, characteristic, payload, response):
    await asyncio.wait_for(
        client.write_gatt_char(characteristic, payload, response=response),
        timeout=3,
    )
    print(f"TX {characteristic.uuid}: {payload.hex(' ').upper()}, response={response}", flush=True)


def run_com(args, live, stop, enabled):
    import serial

    paused = False
    while not stop.is_set():
        if not enabled.is_set():
            if not paused:
                live.set_message("Порт закрыт. Нажмите «Подключить».")
                paused = True
            time.sleep(0.15)
            continue
        paused = False
        connection = None
        opened = False
        retry = False
        try:
            connection = serial.Serial(baudrate=args.baud, timeout=0.05)
            connection.rts = False
            connection.dtr = True
            connection.port = args.port
            connection.open()
            opened = True
            live.connection_opened()
            live.set_message(f"{args.port}: слушаю кадры. Старт и стоп — кнопками.")
            while not stop.is_set() and enabled.is_set():
                command = live.next_command()
                if command:
                    token, name, payload = command
                    try:
                        connection.write(payload)
                        live.command_written(token, name)
                        print(f"TX {args.port}: {payload.hex(' ').upper()}", flush=True)
                    except Exception as error:
                        live.command_failed(token, error)
                        print(f"Команда {name}: {error}", file=sys.stderr, flush=True)
                waiting = connection.in_waiting
                chunk = connection.read(min(max(waiting, 1), 8192))
                if chunk:
                    live.add(chunk)
        except (serial.SerialException, OSError) as error:
            retry = enabled.is_set() and not stop.is_set()
            live.set_message(f"Ошибка {args.port}: {error}" + (" | повтор через 3 с" if retry else ""))
            print("Закройте другие программы, использующие порт, и проверьте --port/--baud.", file=sys.stderr)
        finally:
            if connection is not None and connection.is_open:
                try:
                    connection.write(STOP_CMD)
                    print(f"TX {args.port}: {STOP_CMD.hex(' ').upper()}", flush=True)
                except Exception as error:
                    print(f"Стоп перед закрытием порта не передан: {error}", file=sys.stderr, flush=True)
                try:
                    connection.close()
                except OSError:
                    pass
            if opened:
                live.connection_closed()
        if stop.is_set():
            break
        if retry:
            for _ in range(30):
                if stop.is_set() or not enabled.is_set():
                    break
                time.sleep(0.1)


async def run_ble_session(args, live, stop, enabled, bleak_module):
    live.set_message(f"Поиск {args.address or args.name}")
    discovered = await interruptible(
        bleak_module.BleakScanner.discover(timeout=args.scan_seconds, return_adv=True),
        stop, enabled, args.scan_seconds + 5)
    for device, advertisement in discovered.values():
        print(f"  BLE: {advertisement.local_name or device.name or '(без имени)'} | {device.address}", flush=True)
    device = select_device(discovered, args.name, args.address)
    disconnected = asyncio.Event()
    client = bleak_module.BleakClient(device, disconnected_callback=lambda _client: disconnected.set())
    write = None
    use_response = False
    write_ready = False
    rx_active = False
    try:
        live.set_message(f"Подключение к {device.name or args.name} ({device.address})")
        await interruptible(client.connect(), stop, enabled, 30.0)
        notify = select_characteristic(client.services, args.char_uuid, args.service_uuid)
        write = select_write(client.services, args.write_uuid, args.service_uuid, notify)
        use_response = response_mode(write, args.write_mode)
        write_ready = True
        live.connection_opened()
        rx_active = True

        def on_notification(_sender, payload):
            if rx_active and not stop.is_set() and enabled.is_set():
                live.add(bytes(payload))

        await interruptible(client.start_notify(notify, on_notification), stop, enabled, 15.0)
        mtu = getattr(client, "mtu_size", "?")
        live.set_message(
            f"BLE подключён: {args.name} | FFF1 Notify / FFF2 Write | MTU {mtu}. "
            "Измерения только кнопкой «Старт»."
        )
        while not stop.is_set() and enabled.is_set() and client.is_connected and not disconnected.is_set():
            command = live.next_command()
            if command:
                token, name, payload = command
                try:
                    await send_command(client, write, payload, use_response)
                    live.command_written(token, name)
                except Exception as error:
                    live.command_failed(token, error)
                    print(f"Команда {name}: {error}", file=sys.stderr, flush=True)
            await asyncio.sleep(0.03)
        if not stop.is_set() and enabled.is_set():
            raise ConnectionError("BLE-соединение потеряно. Автоматический старт не выполняется.")
    finally:
        rx_active = False
        if write_ready and client.is_connected:
            try:
                await send_command(client, write, STOP_CMD, use_response)
            except Exception as error:
                print(f"Стоп перед отключением не передан: {error}", file=sys.stderr, flush=True)
        live.connection_closed()
        try:
            await asyncio.wait_for(client.disconnect(), timeout=5.0)
        except Exception as error:
            print(f"Завершение BLE-сессии: {error}", file=sys.stderr, flush=True)


async def ble_loop(args, live, stop, enabled, bleak_module):
    paused = False
    while not stop.is_set():
        if not enabled.is_set():
            if not paused:
                live.set_message("Отключено. Нажмите «Подключить».")
                paused = True
            await asyncio.sleep(0.15)
            continue
        paused = False
        try:
            await run_ble_session(args, live, stop, enabled, bleak_module)
        except OperationInterrupted:
            pass
        except Exception as error:
            live.set_message(f"BLE: {error} | повтор через 3 с")
            for _ in range(30):
                if stop.is_set() or not enabled.is_set():
                    break
                await asyncio.sleep(0.1)


def ble_thread(args, live, stop, enabled):
    try:
        import bleak
        asyncio.run(ble_loop(args, live, stop, enabled, bleak))
    except Exception as error:
        live.set_message(f"BLE-поток завершён: {error}. Перезапустите приложение.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ble", action="store_true", help="Слушать notify FFF1 модуля E104-BT5032A")
    parser.add_argument("--port", help="COM-порт, по умолчанию COM9")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Скорость UART, по умолчанию 115200")
    parser.add_argument("--name", default=DEFAULT_NAME, help="Имя BLE-устройства")
    parser.add_argument("--address", help="Адрес из результатов сканирования вместо выбора по имени")
    parser.add_argument("--char-uuid", default=DEFAULT_CHARACTERISTIC, help="Notify UUID: по умолчанию fff1")
    parser.add_argument("--write-uuid", default=DEFAULT_WRITE, help="Write UUID команд: по умолчанию fff2")
    parser.add_argument("--write-mode", choices=("auto", "response", "no-response"), default="auto")
    parser.add_argument("--service-uuid", help="UUID сервиса, если Notify UUID встречается несколько раз")
    parser.add_argument("--scan-seconds", type=float, default=6.0)
    parser.add_argument("--window", type=float, default=5.0)
    args = parser.parse_args()
    if args.ble and args.port:
        parser.error("Нужен один источник: --ble или --port COMx")
    if not args.ble and not args.port:
        args.port = DEFAULT_PORT
    if (args.baud <= 0 or not math.isfinite(args.scan_seconds) or args.scan_seconds <= 0
            or not math.isfinite(args.window) or args.window <= 0):
        parser.error("Скорость, окно и время сканирования должны быть положительными.")
    if args.ble:
        try:
            args.char_uuid = normalize_uuid(args.char_uuid)
            args.write_uuid = normalize_uuid(args.write_uuid)
            if args.service_uuid:
                args.service_uuid = normalize_uuid(args.service_uuid)
        except ValueError:
            parser.error("Неверный UUID характеристики или сервиса.")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button

    live = Live(args.window)
    stop, enabled = threading.Event(), threading.Event()
    enabled.set()
    if args.ble:
        worker = threading.Thread(target=ble_thread, args=(args, live, stop, enabled), daemon=True)
    else:
        worker = threading.Thread(target=run_com, args=(args, live, stop, enabled), daemon=True)
    worker.start()

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 8.2))
    figure.subplots_adjust(bottom=0.42)
    emg_line, = axes[0].plot([], [], linewidth=0.8)
    angle_line, = axes[1].plot([], [], linewidth=1.0)
    axes[0].set_ylabel("ЭМГ, код АЦП")
    axes[1].set_ylabel("Угол, градусы")
    axes[1].set_xlabel("Время кадра, с")
    axes[1].set_ylim(0, 180)
    notice = figure.text(0.04, 0.075, "", va="bottom", fontsize=8)
    labels = ("Старт", "Стоп", "Подключить", "Отключить")
    buttons = [
        Button(figure.add_axes([0.04 + index * 0.24, 0.02, 0.22, 0.042]), label)
        for index, label in enumerate(labels)
    ]
    buttons[0].on_clicked(lambda _event: live.request("start"))
    buttons[1].on_clicked(lambda _event: live.request("stop"))
    buttons[2].on_clicked(lambda _event: enabled.set())
    buttons[3].on_clicked(lambda _event: enabled.clear())

    def update(_frame):
        emg, angle, samples, frames, message, power, event, timing, controls = live.snapshot()
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
            f"{controls['measurement']}\n"
            f"{describe_gap(timing, args.ble)}\n"
            f"{describe_ready(timing)}\n"
            f"{describe_power(power)}\n"
            f"{describe_event(event)}\n"
            f"{controls['warning']}"
        )
        for index, allowed in ((0, controls["can_start"]), (1, controls["can_stop"])):
            buttons[index].set_active(allowed)
            buttons[index].label.set_alpha(1 if allowed else 0.35)
        return emg_line, angle_line, notice

    animation = FuncAnimation(figure, update, interval=40, blit=False, cache_frame_data=False)
    figure.canvas.mpl_connect("close_event", lambda _event: stop.set())
    try:
        plt.show()
    finally:
        stop.set()
        enabled.clear()
        if animation.event_source is not None:
            animation.event_source.stop()
        worker.join(timeout=8)
        del animation


if __name__ == "__main__":
    sys.exit(main())
