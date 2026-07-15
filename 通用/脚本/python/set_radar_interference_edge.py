import argparse
import struct
import time

import serial


FRAME_HEAD = b"\x53\x59"
FRAME_TAIL = b"\x54\x43"

EDGE_MODES = {
    1: "left edge 0.5m, front 4m",
    2: "right edge 0.5m, front 4m",
    3: "hotel corridor, left/right 0.75m",
    4: "no edge boundary",
}


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


def parse_edge_mode(frame):
    if frame["control"] != 0x07 or frame["command"] not in (0x1A, 0x9A):
        return None
    if not frame["data"]:
        return None
    mode = frame["data"][0]
    return {"mode": mode, "description": EDGE_MODES.get(mode, "unknown")}


def print_frames(label, frames):
    for frame in frames:
        parsed = parse_edge_mode(frame)
        suffix = f" parsed={parsed}" if parsed is not None else ""
        print(
            f"{label}: {hex_with_spaces(frame['raw'])} "
            f"{'OK' if frame['checksum_ok'] else 'CHECKSUM_ERR'}{suffix}"
        )


def main():
    arg_parser = argparse.ArgumentParser(
        description="Set Yunfan R60BMP1 interference edge mode."
    )
    arg_parser.add_argument("--port", default="COM3", help="Serial port, default COM3")
    arg_parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    arg_parser.add_argument(
        "--mode",
        type=int,
        default=4,
        choices=sorted(EDGE_MODES),
        help="Interference edge mode. Default 4 means no edge boundary.",
    )
    arg_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print frames; do not open serial port.",
    )
    args = arg_parser.parse_args()

    set_frame = build_frame(0x07, 0x1A, bytes([args.mode]))
    query_frame = build_frame(0x07, 0x9A, b"\x0F")

    print(f"Port: {args.port}, baud: {args.baud}")
    print(f"Interference edge mode: {args.mode} ({EDGE_MODES[args.mode]})")
    print(f"Set edge frame: {hex_with_spaces(set_frame)}")
    print(f"Verify query: {hex_with_spaces(query_frame)}")

    if args.dry_run:
        return

    parser = FrameParser()
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        ser.write(set_frame)
        ser.flush()
        print_frames("Set edge response", read_frames(ser, parser, 1.0))

        ser.write(query_frame)
        ser.flush()
        query_frames = read_frames(ser, parser, 2.0)

    print_frames("Query response", query_frames)

    verified = [
        parsed
        for parsed in (parse_edge_mode(frame) for frame in query_frames if frame["checksum_ok"])
        if parsed and parsed["mode"] == args.mode
    ]
    if not verified:
        raise SystemExit(f"Edge mode verification failed: expected mode {args.mode}")

    print(f"Verified interference edge mode: {verified[-1]}")


if __name__ == "__main__":
    main()
