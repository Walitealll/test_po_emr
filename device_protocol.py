"""Протокол test_po_emr, commit 14b53d7727a5d7b616cbcb4f15316407c9ef009c.

FFF1 -> приложение; FFF2 <- команды. Все числа little-endian.
Файлы firmware_reference_test_po_emr не нужны для запуска приложения.
"""

from collections import deque
import math
import queue
import struct
import threading
import time


FRAME_LEN = {1: 4, 2: 3, 3: 47, 4: 2}
EMG_COUNT = 20
SAMPLE_TYPE = 3
START = bytes.fromhex("04 01")
STOP = bytes.fromhex("04 02")
CHARGING = {0: "Кабель отключён", 1: "Кабель подключён, заряд идёт", 2: "Кабель подключён, заряд окончен"}
ERRORS = {0: "ошибки нет", 1: "BNO055: ошибка датчика, угол недостоверен", 2: "BNO055: таймаут I2C"}


def decode_frame(raw):
    if not raw or FRAME_LEN.get(raw[0]) != len(raw):
        raise ValueError("Неверный тип или размер кадра")
    kind = raw[0]
    if kind == 3:
        fields = struct.unpack_from("<20HHI", raw, 1)
        return dict(type=kind, emg=fields[:20], angle_x10=fields[20], time_ms=fields[21])
    if kind == 1:
        voltage, charge = struct.unpack_from("<HB", raw, 1)
        return dict(type=kind, voltage_mV=voltage, charge=charge)
    if kind == 2:
        return dict(type=kind, event=raw[1], code=raw[2])
    return dict(type=kind, command=raw[1])


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.skipped = 0

    def reset(self):
        self.buffer.clear()
        self.skipped = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames, offset = [], 0
        while offset < len(self.buffer):
            size = FRAME_LEN.get(self.buffer[offset])
            if size is None:
                self.skipped += 1
                offset += 1
                continue
            if len(self.buffer) - offset < size:
                break
            frames.append(decode_frame(self.buffer[offset:offset + size]))
            offset += size
        del self.buffer[:offset]
        return frames


class DeviceClock:
    """Разворачивает millis uint32, не смешивает дубликаты/старые кадры."""

    def __init__(self):
        self.raw = None
        self.extended = None

    def unwrap(self, raw):
        if self.raw is None:
            delta = None
            self.extended = raw
        else:
            delta = (raw - self.raw) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                raise ValueError("Дубликат или немонотонная метка времени")
            self.extended += delta
        self.raw = raw
        return self.extended, delta


class ProtocolState:
    """Общее состояние GUI/BLE под одним lock. Команды исполняет BLE-поток."""

    def __init__(self, window=5.0):
        self.window = window
        self.lock = threading.RLock()
        self.commands = queue.Queue()
        self.serial = 0
        self.connected = False
        self.link = "Поиск BLE-устройства…"
        self.emg, self.angle = deque(), deque()
        self.last_device = self.last_host = None
        self.clock = DeviceClock()
        self.notices = deque(maxlen=12)
        self._session_reset()

    def _session_reset(self):
        self.parser = FrameParser()
        self.clock = DeviceClock()
        self.emg.clear()
        self.angle.clear()
        self.last_device = self.last_host = None
        self.voltage_mV = self.charge = self.power_host = None
        self.low_battery = False
        self.cable_event = False
        self.mode = "idle"
        self.measurement = "Измерения не запущены"
        self.command_host = None
        self.frames = self.measurement_frames = self.samples = self.notify_count = 0
        self.gaps = self.bad_timestamps = self.bad_angles = self.echoes = 0
        self.last_size = 0
        self.last_delta = None
        self.last_notify_host = None
        self.notify_log = deque()
        self.warning = ""
        self.serial += 1

    def status(self, text):
        with self.lock:
            self.link = text
        print(text, flush=True)

    def _notice(self, text):
        self.notices.append(text)
        print(text, flush=True)

    def connection_opened(self):
        with self.lock:
            self._session_reset()
            self.connected = True

    def connection_closed(self):
        with self.lock:
            self.connected = False
            self.serial += 1
            self.mode = "disconnected"
            self.measurement = "BLE отключён; состояние измерений не подтверждено"
            self.parser.reset()

    def _blocked(self):
        return self.low_battery or self.cable_event or self.charge not in (None, 0)

    def request(self, command, now=None):
        if command not in ("start", "stop"):
            raise ValueError("Неизвестная команда")
        with self.lock:
            if not self.connected:
                self.warning = "Сначала подключитесь к устройству"
                return False
            if command == "start" and (self._blocked() or self.mode in ("measuring", "start_pending", "stop_pending")):
                self.warning = "START сейчас недоступен: проверьте кабель, заряд и состояние измерений"
                return False
            if command == "stop" and self.mode == "stop_pending":
                return False
            self.serial += 1
            if command == "start":
                self.warning = ""
            self.command_host = time.monotonic() if now is None else now
            self.mode = "start_pending" if command == "start" else "stop_pending"
            self.measurement = "START в очереди; ждём поток измерений" if command == "start" else "STOP в очереди; ждём событие остановки"
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
                return token, command, START if command == "start" else STOP

    def command_written(self, token, command):
        with self.lock:
            expected = "start_pending" if command == "start" else "stop_pending"
            if token == self.serial and self.mode == expected:
                self.measurement = ("04 01 передано; ожидаем измерения (отдельного ACK нет)" if command == "start"
                                    else "04 02 передано; ожидаем событие 02 03 …")

    def command_failed(self, token, error):
        with self.lock:
            if token == self.serial:
                self.mode = "unknown"
                self.measurement = "Команда не подтверждена; состояние устройства неизвестно"
                self.warning = f"Ошибка записи: {error}"

    def _stopped(self, message):
        self.serial += 1
        self.mode = "idle"
        self.measurement = message
        self.command_host = None

    def feed(self, payload, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            self.notify_count += 1
            self.last_size = len(payload)
            self.last_notify_host = now
            self.notify_log.append((now, len(payload)))
            while self.notify_log and now - self.notify_log[0][0] > 1:
                self.notify_log.popleft()
            for frame in self.parser.feed(payload):
                self.frames += 1
                kind = frame["type"]
                if kind == 1:
                    self.voltage_mV, self.charge, self.power_host = frame["voltage_mV"], frame["charge"], now
                    if self.charge == 0:
                        was_cable = self.cable_event
                        self.cable_event = False
                        if was_cable and not self.low_battery and self.mode == "idle":
                            self.measurement = "Кабель отключён: готово к START"
                    elif self.charge in (1, 2):
                        self.cable_event = True
                        self._stopped("Кабель подключён: измерения запрещены")
                    else:
                        self.warning = f"Неизвестный код зарядки: {self.charge}"
                elif kind == 2:
                    event, code = frame["event"], frame["code"]
                    if event == 1:
                        self.cable_event = True
                        self._stopped("Кабель вставлен: измерения запрещены")
                    elif event == 2:
                        self.low_battery = True
                        self._stopped("Принудительная остановка: низкий заряд")
                    elif event == 3:
                        self._stopped("Измерение остановлено: " + ERRORS.get(code, f"неизвестная ошибка {code}"))
                        if code:
                            self.warning = ERRORS.get(code, f"Неизвестная ошибка {code}")
                    else:
                        self.warning = f"Неизвестное событие {event}, ошибка {code}"
                    self._notice(f"02 {event:02X} {code:02X}: {self.measurement}")
                elif kind == 3:
                    self._add_measurement(frame, now)
                else:
                    # Эхо 04 xx НЕ является подтверждением выполнения команды.
                    self.echoes += 1

    def _add_measurement(self, frame, now):
        try:
            timestamp, delta = self.clock.unwrap(frame["time_ms"])
        except ValueError as error:
            self.bad_timestamps += 1
            self.warning = str(error) + "; переподключитесь при перезагрузке платы"
            return
        self.last_delta = delta
        end = timestamp / 1000.0
        discontinuity = delta is not None and delta != 10
        if discontinuity:
            self.gaps += 1
            self.emg.append((math.nan, math.nan))
            self.angle.append((math.nan, math.nan))
        self.emg.extend((end - (19 - index) * 0.0005, value) for index, value in enumerate(frame["emg"]))
        angle = frame["angle_x10"] / 10.0
        if not 0 <= angle <= 180:
            self.bad_angles += 1
            self.warning = f"Угол вне протокола: {angle:g}°"
            angle = math.nan
        self.angle.append((end, angle))
        self.last_device, self.last_host = end, now
        self.measurement_frames += 1
        self.samples += EMG_COUNT
        # Кадры, находившиеся в очереди BLE до STOP, не отменяют stop_pending.
        if self.mode != "stop_pending" and not self._blocked():
            self.mode = "measuring"
            self.measurement = "Идут измерения (подтверждено кадрами 03)"
        elif self._blocked():
            self.warning = "Пришли измерения при запрете питания; проверьте прошивку"
        self._trim(end)

    def _trim(self, end):
        for points in (self.emg, self.angle):
            while points and (not math.isfinite(points[0][0]) or points[0][0] < end - self.window):
                points.popleft()

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            end = None if self.last_device is None else self.last_device + max(0, now - self.last_host)
            if end is not None:
                self._trim(end)
            if self.command_host is not None and now - self.command_host > 3:
                if self.mode == "start_pending":
                    self.mode = "start_unconfirmed"
                    self.measurement = "Нет подтверждения START: кадры 03 не поступают; проверьте кабель/датчик"
                elif self.mode == "stop_pending":
                    self.measurement = "Нет подтверждения STOP: событие 02 03 не получено"
            battery = "АКБ: ещё нет телеметрии (период 10 с)"
            if self.voltage_mV is not None:
                battery = f"АКБ: {self.voltage_mV / 1000:.3f} В ({self.voltage_mV} мВ); " + CHARGING.get(self.charge, "неизвестно")
                battery += f" [код {self.charge}; {max(0, now - self.power_host):.0f} с назад]"
            if self.cable_event and self.charge in (None, 0):
                battery += " | Кабель вставлен по событию 02 01; телеметрия ещё не обновилась"
            emg_vals = [y for _, y in self.emg if math.isfinite(y)]
            angle_vals = [y for _, y in self.angle if math.isfinite(y)]
            while self.notify_log and now - self.notify_log[0][0] > 1:
                self.notify_log.popleft()
            return dict(emg=list(self.emg), angle=list(self.angle), end=end, connected=self.connected,
                        link=self.link, mode=self.mode, measurement=self.measurement, battery=battery,
                        warning=self.warning, frames=self.frames, samples=self.samples,
                        measurement_frames=self.measurement_frames, notifications=self.notify_count,
                        last_size=self.last_size, pending=len(self.parser.buffer), skipped=self.parser.skipped,
                        gaps=self.gaps, bad_timestamps=self.bad_timestamps,
                        emg_min=min(emg_vals) if emg_vals else None,
                        emg_max=max(emg_vals) if emg_vals else None,
                        emg_over=sum(y > 4095 for y in emg_vals),
                        angle_min=min(angle_vals) if angle_vals else None,
                        angle_max=max(angle_vals) if angle_vals else None,
                        last_delta=self.last_delta,
                        window_gaps=sum(not math.isfinite(x) for x, _ in self.emg),
                        bytes_per_s=sum(size for _, size in self.notify_log),
                        notify_age_ms=None if self.last_notify_host is None else max(0, (now - self.last_notify_host) * 1000),
                        can_start=self.connected and not self._blocked() and self.mode not in ("measuring", "start_pending", "stop_pending"),
                        can_stop=self.connected and self.mode != "stop_pending")
