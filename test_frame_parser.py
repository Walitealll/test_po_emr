import struct
import unittest

from frame_parser import EMG_COUNT, FrameParser, decode_frame


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


if __name__ == "__main__":
    unittest.main()
