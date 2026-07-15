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
        if ser.in_waiting:
            frames.extend(parser.feed(ser.read(ser.in_waiting)))
        time.sleep(0.05)
    return frames


def parse_range(frame):
    if frame["control"] != 0x07 or frame["command"] not in (0x09, 0x89):
        return None
    data = frame["data"]
    if len(data) == 1:
        return {"auto_range": data[0] == 1}
    if len(data) >= 9:
        return {
            "mode": data[0],
            "x_pos_cm": struct.unpack(">H", data[1:3])[0],
            "x_neg_cm": struct.unpack(">H", data[3:5])[0],
            "y_pos_cm": struct.unpack(">H", data[5:7])[0],
            "y_neg_cm": struct.unpack(">H", data[7:9])[0],
        }
    return None


def print_frames(label, frames):
    for frame in frames:
        parsed = parse_range(frame)
        suffix = f" parsed={parsed}" if parsed is not None else ""
        print(
            f"{label}: {hex_with_spaces(frame['raw'])} "
            f"{'OK' if frame['checksum_ok'] else 'CHECKSUM_ERR'}{suffix}"
        )


def main():
    arg_parser = argparse.ArgumentParser(
        description="Set Yunfan R60BMP1 radar detection boundary."
    )
    arg_parser.add_argument("--port", default="COM3", help="Serial port, default COM3")
    arg_parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    arg_parser.add_argument("--x-pos-cm", type=int, default=165)
    arg_parser.add_argument("--x-neg-cm", type=int, default=165)
    arg_parser.add_argument("--y-pos-cm", type=int, default=500)
    arg_parser.add_argument("--y-neg-cm", type=int, default=0)
    arg_parser.add_argument(
        "--keep-auto",
        action="store_true",
        help="Do not disable automatic range before setting fixed range.",
    )
    args = arg_parser.parse_args()

    values = [args.x_pos_cm, args.x_neg_cm, args.y_pos_cm, args.y_neg_cm]
    if any(value < 0 or value > 65535 for value in values):
        raise SystemExit("Range values must fit uint16 cm")

    disable_auto_frame = build_frame(0x07, 0x0C, b"\x00")
    range_data = (
        b"\x00"
        + struct.pack(">H", args.x_pos_cm)
        + struct.pack(">H", args.x_neg_cm)
        + struct.pack(">H", args.y_pos_cm)
        + struct.pack(">H", args.y_neg_cm)
    )
    set_range_frame = build_frame(0x07, 0x09, range_data)
    query_range_frame = build_frame(0x07, 0x89, b"\x0F")

    print(f"Port: {args.port}, baud: {args.baud}")
    print(
        "Boundary cm: "
        f"X+={args.x_pos_cm}, X-={args.x_neg_cm}, "
        f"Y+={args.y_pos_cm}, Y-={args.y_neg_cm}"
    )
    if not args.keep_auto:
        print(f"Disable auto range frame: {hex_with_spaces(disable_auto_frame)}")
    print(f"Set range frame: {hex_with_spaces(set_range_frame)}")
    print(f"Verify query: {hex_with_spaces(query_range_frame)}")

    parser = FrameParser()
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        if not args.keep_auto:
            ser.write(disable_auto_frame)
            ser.flush()
            print_frames("Disable auto response", read_frames(ser, parser, 1.0))

        ser.write(set_range_frame)
        ser.flush()
        print_frames("Set range response", read_frames(ser, parser, 1.0))

        ser.write(query_range_frame)
        ser.flush()
        query_frames = read_frames(ser, parser, 2.0)

    print_frames("Query response", query_frames)

    fixed_ranges = [
        parsed
        for parsed in (parse_range(frame) for frame in query_frames if frame["checksum_ok"])
        if parsed and "x_pos_cm" in parsed
    ]
    if not fixed_ranges:
        raise SystemExit("Boundary verification failed: no fixed range response")

    expected = {
        "mode": 0,
        "x_pos_cm": args.x_pos_cm,
        "x_neg_cm": args.x_neg_cm,
        "y_pos_cm": args.y_pos_cm,
        "y_neg_cm": args.y_neg_cm,
    }
    if expected not in fixed_ranges:
        raise SystemExit(f"Boundary verification failed: expected {expected}")

    print(f"Verified boundary: {expected}")


if __name__ == "__main__":
    main()
