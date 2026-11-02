"""BLE EMG + угол: протокол FFF1/FFF2, 20 отсчётов в 47-байтовом кадре.

Запуск: start_ble_protocol.cmd. Измерения запускаются ТОЛЬКО кнопкой START.
Старые программы для сырых 80-байтовых блоков не изменены.
"""

import argparse
import asyncio
import math
import sys
import threading
import time
from uuid import UUID

from device_protocol import ProtocolState, STOP


def normalize_uuid(value):
    value = value.strip().lower().removeprefix("0x")
    if len(value) in (4, 8):
        value = f"{int(value, 16):08x}-0000-1000-8000-00805f9b34fb"
    return str(UUID(value))


class Interrupted(Exception):
    pass


async def interruptible(awaitable, stop, enabled, timeout):
    task = asyncio.ensure_future(awaitable)
    deadline = time.monotonic() + timeout
    try:
        while not task.done():
            if stop.is_set() or not enabled.is_set():
                raise Interrupted()
            if time.monotonic() > deadline:
                raise TimeoutError("Таймаут BLE-операции")
            await asyncio.wait({task}, timeout=0.1)
        return await task
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


def choose_device(discovered, args):
    found = []
    for device, adv in discovered.values():
        name = adv.local_name or device.name or ""
        print(f"BLE: {name} | {device.address}", flush=True)
        if ((args.address and args.address.casefold() == device.address.casefold()) or
                (not args.address and name.casefold() == args.name.casefold())):
            found.append(device)
    if len(found) != 1:
        raise LookupError(f"Устройств {args.address or args.name}: {len(found)}. Для одинаковых имён задайте --address.")
    return found[0]


def choose_characteristics(services, args):
    notify, write = [], []
    descriptions = []
    for service in services:
        for char in service.characteristics:
            descriptions.append(f"{char.uuid} [{','.join(char.properties)}] service={service.uuid}")
            if args.service_uuid and normalize_uuid(service.uuid) != normalize_uuid(args.service_uuid):
                continue
            if normalize_uuid(char.uuid) == normalize_uuid(args.notify_uuid) and set(char.properties) & {"notify", "indicate"}:
                notify.append(char)
            if normalize_uuid(char.uuid) == normalize_uuid(args.write_uuid) and set(char.properties) & {"write", "write-without-response"}:
                write.append(char)
    if len(notify) == 1:
        # FFF2 выбираем из того же сервиса, что и FFF1, если это возможно.
        same = [char for char in write if normalize_uuid(char.service_uuid) == normalize_uuid(notify[0].service_uuid)]
        if same:
            write = same
    if len(notify) != 1 or len(write) != 1:
        raise LookupError("Нужны уникальные FFF1 Notify и FFF2 Write; проверьте --service-uuid. " + "; ".join(descriptions))
    return notify[0], write[0]


def response_mode(characteristic, args):
    props = set(characteristic.properties)
    result = "write" in props if args.write_mode == "auto" else args.write_mode == "response"
    required = "write" if result else "write-without-response"
    if required not in props:
        raise ValueError(f"FFF2 не поддерживает {required}")
    return result


async def send_command(client, characteristic, payload, response):
    # Отправляем два байта 04 01 / 04 02, НЕ ASCII-строку '0401'.
    await asyncio.wait_for(client.write_gatt_char(characteristic, payload, response=response), timeout=3)
    print(f"TX {characteristic.uuid}: {payload.hex(' ').upper()}, response={response}", flush=True)


async def run_session(args, state, stop, enabled, bleak):
    state.status(f"Поиск {args.address or args.name}…")
    discovered = await interruptible(bleak.BleakScanner.discover(timeout=args.scan_seconds, return_adv=True),
                                      stop, enabled, args.scan_seconds + 5)
    device = choose_device(discovered, args)
    disconnected = asyncio.Event()
    client = bleak.BleakClient(device, disconnected_callback=lambda _client: disconnected.set())
    rx_active = False
    write = None
    response = False
    write_ready = False
    try:
        state.status(f"Подключение к {device.address}…")
        await interruptible(client.connect(), stop, enabled, 30)
        notify, write = choose_characteristics(client.services, args)
        response = response_mode(write, args)
        write_ready = True
        state.connection_opened()
        rx_active = True

        def on_notify(_char, payload):
            if rx_active:
                state.feed(payload)

        await interruptible(client.start_notify(notify, on_notify), stop, enabled, 15)
        state.status(f"BLE подключён: {args.name} | FFF1 Notify / FFF2 Write | MTU {getattr(client, 'mtu_size', '?')}")
        while not stop.is_set() and enabled.is_set() and client.is_connected and not disconnected.is_set():
            command = state.next_command()
            if command:
                token, name, payload = command
                try:
                    await send_command(client, write, payload, response)
                    state.command_written(token, name)
                except Exception as error:
                    state.command_failed(token, error)
                    print(f"Команда {name}: {error}", file=sys.stderr, flush=True)
            await asyncio.sleep(0.03)
        if not stop.is_set() and enabled.is_set():
            raise ConnectionError("BLE-связь потеряна. Автоматический START не выполняется.")
    finally:
        # При явном отключении/закрытии окна пытаемся безопасно остановить
        # измерения ДО разрыва BLE. ATT-ответ не означает ACK команды платы.
        if write_ready and client.is_connected:
            try:
                await send_command(client, write, STOP, response)
            except Exception as error:
                print(f"STOP перед отключением не подтверждён: {error}", file=sys.stderr, flush=True)
        rx_active = False
        state.connection_closed()
        try:
            await asyncio.wait_for(client.disconnect(), timeout=5)
        except Exception as error:
            print(f"Завершение BLE: {error}", file=sys.stderr, flush=True)


async def ble_loop(args, state, stop, enabled, bleak):
    paused = False
    while not stop.is_set():
        if not enabled.is_set():
            if not paused:
                state.status("BLE отключён. Нажмите «Подключить BLE».")
                paused = True
            await asyncio.sleep(0.1)
            continue
        paused = False
        try:
            await run_session(args, state, stop, enabled, bleak)
        except Interrupted:
            pass
        except Exception as error:
            state.status(f"BLE: {error} | повтор через 3 с")
            for _ in range(30):
                if stop.is_set() or not enabled.is_set():
                    break
                await asyncio.sleep(0.1)


def ble_worker(args, state, stop, enabled):
    try:
        import bleak
        asyncio.run(ble_loop(args, state, stop, enabled, bleak))
    except Exception as error:
        state.status(f"BLE-поток завершён: {error}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="E104-BT5032A")
    parser.add_argument("--address")
    parser.add_argument("--notify-uuid", default="fff1")
    parser.add_argument("--write-uuid", default="fff2")
    parser.add_argument("--service-uuid")
    parser.add_argument("--write-mode", choices=("auto", "response", "no-response"), default="auto")
    parser.add_argument("--window", type=float, default=5)
    parser.add_argument("--scan-seconds", type=float, default=6)
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in (args.window, args.scan_seconds)):
        parser.error("Окно и время поиска должны быть положительными конечными числами")
    try:
        for value in (args.notify_uuid, args.write_uuid, args.service_uuid):
            if value:
                normalize_uuid(value)
    except ValueError:
        parser.error("Неверный UUID")
    try:
        import importlib.util
        if importlib.util.find_spec("bleak") is None:
            raise ImportError("bleak")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button
    except ImportError as error:
        print(f"Не хватает библиотеки: {error}. Запустите start_ble_protocol.cmd", file=sys.stderr)
        return 1

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(12, 9.4))
    figure.canvas.manager.set_window_title("BLE — ЭМГ и угол / протокол устройства")
    figure.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.44, hspace=0.28)
    figure.suptitle(f"{args.name} — ЭМГ и угол, окно {args.window:g} с")
    emg_line, = axes[0].plot([], [], linewidth=0.8, color="#1769aa")
    angle_line, = axes[1].plot([], [], linewidth=1.3, color="#be5918")
    axes[0].set_ylabel("ЭМГ, код АЦП")
    axes[1].set_ylabel("Угол, градусы")
    axes[1].set_xlabel("Время относительно конца окна, с (метки платы)")
    axes[0].set_ylim(-100, 4200)
    axes[1].set_ylim(-5, 185)
    for axis in axes:
        axis.set_xlim(-args.window, 0)
        axis.grid(True, alpha=0.25)
    texts = [figure.text(0.09, y, "", fontsize=9) for y in (0.275, 0.235, 0.195, 0.155, 0.115)]
    diag = [figure.text(0.09, y, "", fontsize=9) for y in (0.395, 0.360, 0.325)]
    warning = figure.text(0.09, 0.075, "", fontsize=9, color="#ac3018")
    labels = ("START измерения", "STOP измерения", "Подключить BLE", "Отключить BLE")
    buttons = [Button(figure.add_axes([0.09 + i * 0.225, 0.015, 0.21, 0.04]), label) for i, label in enumerate(labels)]
    state = ProtocolState(args.window)
    stop, enabled = threading.Event(), threading.Event()
    enabled.set()
    buttons[0].on_clicked(lambda _event: state.request("start"))
    buttons[1].on_clicked(lambda _event: state.request("stop"))
    buttons[2].on_clicked(lambda _event: enabled.set())
    buttons[3].on_clicked(lambda _event: enabled.clear())
    worker = threading.Thread(target=ble_worker, args=(args, state, stop, enabled), daemon=True)
    worker.start()

    def shorten(text, limit=125):
        return text if len(text) <= limit else text[:limit - 1] + "…"

    def update(_frame):
        snapshot = state.snapshot()
        end = snapshot["end"]
        for line, key in ((emg_line, "emg"), (angle_line, "angle")):
            points = snapshot[key]
            line.set_data([x - end for x, _ in points] if end is not None else [], [y for _, y in points])
        peak = max((y for _, y in snapshot["emg"] if math.isfinite(y)), default=4095)
        axes[0].set_ylim(-100, max(4200, peak * 1.03))
        texts[0].set_text(shorten(snapshot["link"]))
        texts[1].set_text(shorten(snapshot["measurement"]))
        texts[2].set_text(snapshot["battery"])
        texts[3].set_text(f"Notify: {snapshot['notifications']}; последний: {snapshot['last_size']} Б; "
                          f"кадров 03: {snapshot['measurement_frames']}; ЭМГ: {snapshot['samples']}; "
                          f"буфер: {snapshot['pending']} Б; разрывы: {snapshot['gaps']}")
        texts[4].set_text("Питание: 01 / 4 Б; события: 02 / 3 Б; измерения: 03 / 47 Б; команды: 04 / 2 Б")

        def span(low, high, digits):
            if low is None:
                return "—"
            return f"{low:.{digits}f} / {high:.{digits}f}"

        step = "—" if snapshot["last_delta"] is None else f"{snapshot['last_delta']} мс"
        age = "—" if snapshot["notify_age_ms"] is None else f"{snapshot['notify_age_ms']:.0f} мс назад"
        diag[0].set_text(shorten(
            f"ЭМГ мин/макс: {span(snapshot['emg_min'], snapshot['emg_max'], 0)}; выше 4095: {snapshot['emg_over']}"))
        diag[1].set_text(shorten(
            f"Угол мин/макс: {span(snapshot['angle_min'], snapshot['angle_max'], 1)}°; "
            f"шаг метки: {step}; иных шагов в окне: {snapshot['window_gaps']}"))
        diag[2].set_text(shorten(
            f"Поток: {snapshot['bytes_per_s']} Б/с; notify: {snapshot['last_size']} Б; последний notify: {age}"))
        problem = snapshot["warning"]
        if snapshot["skipped"]:
            problem += f" | Неизвестных байтов пропущено: {snapshot['skipped']}"
        warning.set_text(shorten(problem))
        for index, allowed in ((0, snapshot["can_start"]), (1, snapshot["can_stop"])):
            buttons[index].set_active(allowed)
            buttons[index].label.set_alpha(1 if allowed else 0.35)
        return emg_line, angle_line, *texts, *diag, warning

    animation = FuncAnimation(figure, update, interval=40, blit=False, cache_frame_data=False)
    figure.canvas.mpl_connect("close_event", lambda _event: stop.set())
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        enabled.clear()
        if animation.event_source is not None:
            animation.event_source.stop()
        worker.join(timeout=10)
        plt.close(figure)
        if worker.is_alive():
            print("BLE-поток ещё завершает работу; STOP не гарантирован при обрыве связи", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
