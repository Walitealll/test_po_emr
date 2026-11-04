import struct
import unittest
from types import SimpleNamespace

from frame_parser import EMG_COUNT, FrameParser, decode_frame
from plot_frame import (
    DEFAULT_BAUD,
    DEFAULT_CHARACTERISTIC,
    DEFAULT_NAME,
    DEFAULT_PORT,
    describe_event,
    describe_power,
    normalize_uuid,
    select_characteristic,
    select_device,
)


def sample_frame(emg, angle, time_ms):
    raw = bytearray(47)
    raw[0] = 0x03
    struct.pack_into("<" + str(EMG_COUNT) + "H", raw, 1, *emg)
    struct.pack_into("<H", raw, 41, angle)
    struct.pack_into("<I", raw, 43, time_ms)
    return bytes(raw)


class FrameParserTest(unittest.TestCase):
    def test_one_sample_frame(self):
        emg = tuple(range(EMG_COUNT))
        frame = decode_frame(sample_frame(emg, 1800, 1000))
        self.assertEqual(frame["emg"], emg)
        self.assertEqual(frame["angle_x10"], 1800)
        self.assertEqual(frame["time_ms"], 1000)

    def test_split_across_reads_and_power_frame_between(self):
        sample = sample_frame((4095,) * EMG_COUNT, 10, 20)
        power = struct.pack("<BHB", 0x01, 3300, 0)
        parser = FrameParser()
        self.assertEqual(parser.feed(sample[:15]), [])
        self.assertEqual(parser.feed(sample[15:40]), [])
        frames = parser.feed(sample[40:] + power)
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0]["type"], 0x03)
        self.assertEqual(frames[0]["emg"][0], 4095)
        self.assertEqual(frames[0]["emg"][-1], 4095)
        self.assertEqual(frames[1], {"type": 0x01, "voltage_mV": 3300, "charge": 0})

    def test_unknown_byte_is_skipped_and_next_frame_survives(self):
        event = bytes((0x02, 1, 0))
        parser = FrameParser()
        frames = parser.feed(b"\xff" + event)
        self.assertEqual(parser.skipped, 1)
        self.assertEqual(frames, [{"type": 0x02, "event": 1, "code": 0}])

    def test_little_endian_not_big(self):
        emg = (0,) * (EMG_COUNT - 1) + (0x0FFF,)
        frame = decode_frame(sample_frame(emg, 1, 5))
        self.assertEqual(frame["emg"][-1], 4095)

    def test_status_lines_name_charge_and_event(self):
        self.assertEqual(describe_power(None), "Зарядка: ещё не приходила")
        self.assertEqual(
            describe_power({"type": 0x01, "voltage_mV": 3700, "charge": 1}),
            "Зарядка: 3700 мВ, заряд идёт (1)",
        )
        self.assertEqual(describe_event(None), "Устройство: событий не было")
        self.assertEqual(
            describe_event({"type": 0x02, "event": 3, "code": 2}),
            "Устройство: измерение остановлено, BNO055 не ответил по I2C (3/2)",
        )


class DeviceSettingsTest(unittest.TestCase):
    def test_defaults_match_com_and_ble_plot(self):
        self.assertEqual(DEFAULT_PORT, "COM9")
        self.assertEqual(DEFAULT_BAUD, 115200)
        self.assertEqual(DEFAULT_NAME, "E104-BT5032A")
        self.assertEqual(DEFAULT_CHARACTERISTIC, "fff1")

    def test_short_and_long_uuid(self):
        expected = "0000fff1-0000-1000-8000-00805f9b34fb"
        for source in ("FFF1", "0xfff1", "0000fff1", expected.upper()):
            self.assertEqual(normalize_uuid(source), expected)

    def test_device_chosen_by_advertised_name_or_address(self):
        device = SimpleNamespace(name="ignored", address="00:11:22:33:44:55")
        discovered = {device.address: (device, SimpleNamespace(local_name="E104-BT5032A"))}
        self.assertIs(select_device(discovered, "e104-bt5032a"), device)
        self.assertIs(select_device(discovered, "ignored", device.address), device)

    def test_one_notify_characteristic(self):
        characteristic = SimpleNamespace(
            uuid=normalize_uuid("fff1"),
            properties=["notify"],
        )
        service = SimpleNamespace(uuid=normalize_uuid("fff0"), characteristics=[characteristic])
        self.assertIs(select_characteristic([service], "FFF1", "fff0"), characteristic)


if __name__ == "__main__":
    unittest.main()
