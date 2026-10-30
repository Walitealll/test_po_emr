"""Разбор потока кадров платы. Длины из docs/protocol.md, little-endian."""

import struct

FRAME_LEN = {
    0x01: 4,
    0x02: 3,
    0x03: 47,
    0x04: 2,
}
EMG_COUNT = 20
SAMPLE_TYPE = 0x03


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.skipped = 0

    def reset(self):
        self.buffer.clear()
        self.skipped = 0

    def feed(self, chunk):
        if chunk:
            self.buffer.extend(chunk)
        frames = []
        while self.buffer:
            kind = self.buffer[0]
            size = FRAME_LEN.get(kind)
            if size is None:
                del self.buffer[0]
                self.skipped += 1
                continue
            if len(self.buffer) < size:
                break
            raw = bytes(self.buffer[:size])
            del self.buffer[:size]
            frames.append(decode_frame(raw))
        return frames


def decode_frame(raw):
    kind = raw[0]
    size = FRAME_LEN.get(kind)
    if size is None or len(raw) != size:
        raise ValueError("Кадр другого размера.")
    if kind == SAMPLE_TYPE:
        emg = struct.unpack_from("<" + str(EMG_COUNT) + "H", raw, 1)
        angle = struct.unpack_from("<H", raw, 1 + EMG_COUNT * 2)[0]
        time_ms = struct.unpack_from("<I", raw, 1 + EMG_COUNT * 2 + 2)[0]
        return {"type": kind, "emg": emg, "angle_x10": angle, "time_ms": time_ms}
    if kind == 0x01:
        voltage, charge = struct.unpack_from("<HB", raw, 1)
        return {"type": kind, "voltage_mV": voltage, "charge": charge}
    if kind == 0x02:
        event, code = struct.unpack_from("<BB", raw, 1)
        return {"type": kind, "event": event, "code": code}
    command = raw[1]
    return {"type": kind, "command": command}
