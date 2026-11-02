"""Тесты протокола и BLE-команд без реальных подключений."""

import asyncio
import math
import struct
import threading
from types import SimpleNamespace
import unittest

import device_protocol as protocol
import plot_ble_protocol as app


def measurement(timestamp=1000, angle=900, values=None):
    return b"\x03" + struct.pack("<20HHI", *(tuple(range(20)) if values is None else values), angle, timestamp)


def power(charge=0, voltage=3800):
    return struct.pack("<BHB", 1, voltage, charge)


def args(**changes):
    values = dict(name="E104-BT5032A", address=None, notify_uuid="fff1", write_uuid="fff2",
                  service_uuid=None, write_mode="auto", scan_seconds=0.01)
    values.update(changes)
    return SimpleNamespace(**values)


def characteristics(props=None):
    service_uuid = app.normalize_uuid("fff0")
    rx = SimpleNamespace(uuid=app.normalize_uuid("fff1"), service_uuid=service_uuid, properties=["notify"])
    tx = SimpleNamespace(uuid=app.normalize_uuid("fff2"), service_uuid=service_uuid,
                         properties=props or ["write", "write-without-response"])
    return [SimpleNamespace(uuid=service_uuid, characteristics=[rx, tx])], rx, tx


class ParserTests(unittest.TestCase):
    def test_all_packet_lengths_and_little_endian(self):
        samples = protocol.decode_frame(measurement(values=[0x0FFF] * 20, angle=1800, timestamp=0x12345678))
        self.assertEqual(samples["emg"], (4095,) * 20)
        self.assertEqual(samples["angle_x10"], 1800)
        self.assertEqual(samples["time_ms"], 0x12345678)
        self.assertEqual(len(measurement()), 47)
        self.assertEqual(protocol.decode_frame(power(1, 4200)), dict(type=1, voltage_mV=4200, charge=1))
        self.assertEqual(protocol.decode_frame(b"\x02\x03\x02"), dict(type=2, event=3, code=2))
        self.assertEqual(protocol.decode_frame(b"\x04\x01"), dict(type=4, command=1))

    def test_every_chunk_size_mixed_frames_no_loss(self):
        stream = power() + b"\xFF\xFE" + measurement(1000) + b"\x02\x03\x00" + measurement(1010) + b"\x04\x02"
        for size in range(1, len(stream) + 1):
            parser, frames = protocol.FrameParser(), []
            for offset in range(0, len(stream), size):
                frames.extend(parser.feed(stream[offset:offset + size]))
            self.assertEqual([frame["type"] for frame in frames], [1, 3, 2, 3, 4])
            self.assertEqual(parser.skipped, 2)
            self.assertEqual(parser.buffer, b"")

    def test_partial_frame_is_retained(self):
        parser = protocol.FrameParser()
        self.assertEqual(parser.feed(measurement()[:46]), [])
        self.assertEqual(len(parser.buffer), 46)
        self.assertEqual(len(parser.feed(measurement()[46:])), 1)

    def test_invalid_decode(self):
        for raw in (b"", b"\xFF", b"\x01\x00", measurement() + b"\x00"):
            with self.assertRaises(ValueError):
                protocol.decode_frame(raw)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.state = protocol.ProtocolState()
        self.state.connection_opened()

    def test_start_exact_bytes_and_confirm_only_by_data(self):
        self.assertTrue(self.state.request("start", now=0))
        token, command, payload = self.state.next_command()
        self.assertEqual(payload, b"\x04\x01")
        self.state.command_written(token, command)
        self.assertEqual(self.state.mode, "start_pending")
        self.state.feed(b"\x04\x01", now=0.1)
        self.assertEqual(self.state.mode, "start_pending")
        self.state.feed(measurement(), now=0.2)
        self.assertEqual(self.state.mode, "measuring")
        # If Notify runs before write_gatt_char returns, do not overwrite confirmation.
        self.state.command_written(token, command)
        self.assertIn("подтверждено", self.state.measurement)

    def test_stop_inflight_data_and_ack(self):
        self.state.feed(measurement(1000), now=1)
        self.assertTrue(self.state.request("stop", now=1))
        token, name, payload = self.state.next_command()
        self.assertEqual(payload, b"\x04\x02")
        self.state.feed(measurement(1010), now=1.01)
        self.assertEqual(self.state.mode, "stop_pending")
        self.state.feed(b"\x02\x03\x00", now=1.02)
        self.assertEqual(self.state.mode, "idle")
        self.state.command_written(token, name)
        self.assertIn("остановлено", self.state.measurement)

    def test_hardware_timestamps_and_angle_endpoint(self):
        self.state.feed(measurement(1000, 1234), now=500)
        result = self.state.snapshot(now=500)
        self.assertEqual(len(result["emg"]), 20)
        self.assertAlmostEqual(result["emg"][0][0], 0.9905)
        self.assertEqual(result["emg"][-1], (1.0, 19))
        self.assertEqual(result["angle"], [(1.0, 123.4)])
        for first, second in zip(result["emg"], result["emg"][1:]):
            self.assertAlmostEqual(second[0] - first[0], 0.0005)

    def test_batched_frames_use_device_time_not_arrival(self):
        self.state.feed(measurement(1000) + power() + measurement(1010), now=50)
        result = self.state.snapshot(50)
        self.assertEqual(result["samples"], 40)
        self.assertAlmostEqual(result["emg"][20][0] - result["emg"][19][0], 0.0005)
        self.assertEqual([x for x, _ in result["angle"]], [1.0, 1.01])

    def test_wraparound_uint32(self):
        self.state.feed(measurement(0xFFFFFFFA), now=1)
        self.state.feed(measurement(4), now=1.01)
        self.assertEqual(self.state.samples, 40)
        self.assertAlmostEqual(self.state.angle[-1][0] - self.state.angle[-2][0], 0.01)

    def test_restart_does_not_reset_device_time(self):
        self.state.feed(measurement(1000), now=1)
        self.state.feed(b"\x02\x03\x00", now=1.01)
        self.state.request("start", now=2)
        self.state.feed(measurement(5000), now=2.01)
        self.assertEqual(self.state.angle[-1][0], 5.0)
        self.assertEqual(self.state.gaps, 1)
        self.assertTrue(any(math.isnan(x) for x, _ in self.state.emg))

    def test_duplicate_and_backward_timestamp_not_plotted(self):
        self.state.feed(measurement(1000) + measurement(1000) + measurement(990), now=1)
        self.assertEqual(self.state.samples, 20)
        self.assertEqual(self.state.bad_timestamps, 2)

    def test_fixed_window_and_idle_scroll(self):
        for index in range(601):
            self.state.feed(measurement(1000 + index * 10), now=10 + index * 0.01)
        result = self.state.snapshot(16)
        self.assertTrue(9999 <= len(result["emg"]) <= 10001)
        self.assertEqual(result["end"], 7)
        self.assertEqual(self.state.snapshot(21.01)["emg"], [])

    def test_charging_mapping_and_cable_blocks_start(self):
        for code, phrase in ((1, "заряд идёт"), (2, "заряд окончен")):
            self.state.feed(power(code), now=1)
            self.assertIn(phrase, self.state.snapshot(1)["battery"])
            self.assertFalse(self.state.request("start", now=1))
        self.state.feed(power(0), now=2)
        self.assertTrue(self.state.snapshot(2)["can_start"])
        self.assertIn("Кабель отключён", self.state.snapshot(2)["battery"])

    def test_cable_event_cancels_queued_start(self):
        self.state.request("start", now=1)
        self.state.feed(b"\x02\x01\x00", now=1)
        self.assertIsNone(self.state.next_command())
        self.assertFalse(self.state.snapshot(1)["can_start"])
        self.state.feed(power(0), now=2)
        self.assertTrue(self.state.snapshot(2)["can_start"])

    def test_low_battery_and_sensor_errors(self):
        self.state.feed(b"\x02\x02\x00", now=1)
        self.assertFalse(self.state.snapshot(1)["can_start"])
        self.assertIn("низкий заряд", self.state.measurement)
        self.state.connection_opened()
        for code, text in ((1, "недостоверен"), (2, "таймаут")):
            self.state.feed(bytes((2, 3, code)), now=2)
            self.assertIn(text, self.state.warning)

    def test_stop_supersedes_not_yet_sent_start(self):
        self.state.request("start", now=1)
        self.state.request("stop", now=1)
        self.assertEqual(self.state.next_command()[2], protocol.STOP)
        self.assertIsNone(self.state.next_command())

    def test_disconnect_cancels_commands_and_reconnect_resets_parser(self):
        self.state.request("start", now=1)
        self.state.feed(measurement()[:30], now=1)
        self.state.connection_closed()
        self.assertIsNone(self.state.next_command())
        self.assertFalse(self.state.snapshot(1)["can_start"])
        self.state.connection_opened()
        self.assertEqual(self.state.samples, 0)
        self.assertEqual(self.state.parser.buffer, b"")

    def test_start_and_stop_timeout_not_false_success(self):
        self.state.request("start", now=0)
        self.assertEqual(self.state.snapshot(4)["mode"], "start_unconfirmed")
        self.state.request("stop", now=5)
        self.assertIn("Нет подтверждения STOP", self.state.snapshot(9)["measurement"])

    def test_voltage_no_invented_percent_and_bad_angle(self):
        self.state.feed(power(0, 3800) + measurement(angle=65535), now=1)
        self.assertIn("3.800 В", self.state.snapshot(1)["battery"])
        self.assertNotIn("%", self.state.snapshot(1)["battery"])
        self.assertTrue(math.isnan(self.state.angle[-1][1]))


class BLETests(unittest.IsolatedAsyncioTestCase):
    def test_gatt_selection_and_response_mode(self):
        services, rx, tx = characteristics()
        self.assertEqual(app.choose_characteristics(services, args()), (rx, tx))
        self.assertTrue(app.response_mode(tx, args()))
        tx.properties = ["write-without-response"]
        self.assertFalse(app.response_mode(tx, args()))
        with self.assertRaises(ValueError):
            app.response_mode(tx, args(write_mode="response"))

    async def test_notify_before_start_exact_commands_and_cleanup_stop(self):
        state = protocol.ProtocolState()
        stop, enabled = threading.Event(), threading.Event()
        enabled.set()
        services, rx, tx = characteristics()
        writes, order = [], []
        device = SimpleNamespace(name="E104-BT5032A", address="00:11:22:33:44:55")

        class Scanner:
            @staticmethod
            async def discover(**kwargs):
                return {device.address: (device, SimpleNamespace(local_name=device.name))}

        class Client:
            def __init__(self, *a, **kw):
                self.services, self.is_connected, self.mtu_size = services, False, 83

            async def connect(self):
                self.is_connected = True

            async def start_notify(self, selected, callback):
                self.callback = callback
                order.append("notify")
                callback(rx, power())
                state.request("start")

            async def write_gatt_char(self, selected, payload, response):
                self.assert_selected = selected is tx
                writes.append((payload, response))
                order.append(payload)
                if payload == protocol.START:
                    self.callback(rx, measurement())
                    state.request("stop")
                else:
                    self.callback(rx, b"\x02\x03\x00")
                    enabled.clear()

            async def disconnect(self):
                self.is_connected = False
                order.append("disconnect")

        await app.run_session(args(), state, stop, enabled, SimpleNamespace(BleakScanner=Scanner, BleakClient=Client))
        self.assertEqual(order[0], "notify")
        self.assertEqual([p for p, _ in writes], [protocol.START, protocol.STOP, protocol.STOP])
        self.assertTrue(all(response for _, response in writes))
        self.assertEqual(state.samples, 20)
        self.assertEqual(order[-1], "disconnect")
        self.assertFalse(state.connected)

    async def test_no_start_automatically_on_connection(self):
        state = protocol.ProtocolState()
        stop, enabled = threading.Event(), threading.Event()
        enabled.set()
        service_list, _, _ = characteristics(["write-without-response"])
        device = SimpleNamespace(name="E104-BT5032A", address="00:00")
        writes = []

        class Scanner:
            @staticmethod
            async def discover(**kwargs):
                return {"00:00": (device, SimpleNamespace(local_name=device.name))}

        class Client:
            def __init__(self, *a, **kw):
                self.services, self.is_connected = service_list, False

            async def connect(self):
                self.is_connected = True

            async def start_notify(self, *a):
                enabled.clear()

            async def write_gatt_char(self, selected, payload, response):
                writes.append((payload, response))

            async def disconnect(self):
                self.is_connected = False

        await app.run_session(args(), state, stop, enabled, SimpleNamespace(BleakScanner=Scanner, BleakClient=Client))
        self.assertEqual(writes, [(protocol.STOP, False)])


if __name__ == "__main__":
    unittest.main()
