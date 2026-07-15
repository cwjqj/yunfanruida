import argparse
import struct
import time

import serial


FRAME_HEAD = b"\x53\x59"
FRAME_TAIL = b"\x54\x43"


def checksum(frame_without_checksum):
    return sum(frame_without_checksum) & 0xFF


def build_frame(control, command, data):
    frame = bytearray()
    frame.extend(FRAME_HEAD)
    frame.append(control)
    frame.append(command)
    frame.extend(struct.pack(">H", len(data)))
    frame.extend(data)
    frame.append(checksum(frame))
    frame.extend(FRAME_TAIL)
    return bytes(frame)


def hex_with_spaces(data):
    return " ".join(f"{byte:02X}" for byte in data)


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, raw_bytes):
        self.buffer.extend(raw_bytes)
        frames = []
        while True:
            frame = self._try_parse_one()
            if frame is None:
                return frames
            frames.append(frame)

    def _try_parse_one(self):
        head_idx = self.buffer.find(FRAME_HEAD)
        if head_idx < 0:
            self.buffer.clear()
            return None
        if head_idx:
            del self.buffer[:head_idx]
        if len(self.buffer) < 9:
            return None

        data_len = struct.unpack(">H", self.buffer[4:6])[0]
        total_len = 2 + 1 + 1 + 2 + data_len + 1 + 2
        if len(self.buffer) < total_len:
            return None

        raw = bytes(self.buffer[:total_len])
        del self.buffer[:total_len]
        data = raw[6 : 6 + data_len]
        expected_checksum = checksum(raw[: 6 + data_len])
        return {
            "raw": raw,
            "control": raw[2],
            "command": raw[3],
            "data": data,
            "checksum_ok": raw[6 + data_len] == expected_checksum,
        }


def read_frames(ser, parser, timeout):
    end_at = time.time() + timeout
    frames = []
    while time.time() < end_at:
        waiting = ser.in_waiting
        if waiting:
            frames.extend(parser.feed(ser.read(waiting)))
        time.sleep(0.05)
    return frames


def parse_height_cm(frame):
    if (
        frame["control"] == 0x06
        and frame["command"] in (0x02, 0x82)
        and len(frame["data"]) >= 2
    ):
        return struct.unpack(">H", frame["data"][:2])[0]
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Set Yunfan R60BMP1 radar installation height."
    )
    parser.add_argument("--port", default="COM3", help="Serial port, default COM3")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--height-cm", type=int, default=200, help="Installation height in cm"
    )
    args = parser.parse_args()

    if not 0 <= args.height_cm <= 65535:
        raise SystemExit("--height-cm must fit uint16")

    height_data = struct.pack(">H", args.height_cm)
    set_frame = build_frame(0x06, 0x02, height_data)
    query_frame = build_frame(0x06, 0x82, b"\x0F")

    print(f"Port: {args.port}, baud: {args.baud}")
    print(f"Set height: {args.height_cm} cm")
    print(f"Set frame: {hex_with_spaces(set_frame)}")
    print(f"Verify query: {hex_with_spaces(query_frame)}")

    parser_state = FrameParser()
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        ser.write(set_frame)
        ser.flush()
        set_responses = read_frames(ser, parser_state, 1.0)
        for frame in set_responses:
            print(
                "Set response:",
                hex_with_spaces(frame["raw"]),
                "OK" if frame["checksum_ok"] else "CHECKSUM_ERR",
            )

        ser.write(query_frame)
        ser.flush()
        query_responses = read_frames(ser, parser_state, 2.0)

    verified_height = None
    for frame in query_responses:
        print(
            "Query response:",
            hex_with_spaces(frame["raw"]),
            "OK" if frame["checksum_ok"] else "CHECKSUM_ERR",
        )
        height = parse_height_cm(frame)
        if frame["checksum_ok"] and height is not None:
            verified_height = height

    if verified_height != args.height_cm:
        raise SystemExit(
            f"Height verification failed: expected {args.height_cm} cm, "
            f"got {verified_height!r}"
        )

    print(f"Verified installation height: {verified_height} cm")


if __name__ == "__main__":
    main()
