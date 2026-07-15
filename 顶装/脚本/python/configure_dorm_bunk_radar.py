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


def pack_u16(value):
    if not 0 <= value <= 65535:
        raise ValueError(f"{value} does not fit uint16")
    return struct.pack(">H", value)


def pack_i16(value):
    if not -32768 <= value <= 32767:
        raise ValueError(f"{value} does not fit int16")
    return struct.pack(">h", value)


def pack_u32(value):
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"{value} does not fit uint32")
    return struct.pack(">I", value)


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


def parse_i16_scaled_100(data):
    return struct.unpack(">h", data)[0] / 100.0


def parse_frame(frame):
    data = frame["data"]
    control = frame["control"]
    command = frame["command"]

    if control == 0x06 and command in (0x01, 0x81) and len(data) >= 6:
        return {
            "angle_x_deg": parse_i16_scaled_100(data[0:2]),
            "angle_y_deg": parse_i16_scaled_100(data[2:4]),
            "angle_z_deg": parse_i16_scaled_100(data[4:6]),
        }
    if control == 0x06 and command in (0x02, 0x82) and len(data) >= 2:
        return {"height_cm": struct.unpack(">H", data[:2])[0]}
    if control == 0x07 and command in (0x09, 0x89):
        if len(data) == 1:
            return {"range_mode": data[0]}
        if len(data) >= 9:
            return {
                "range_mode": data[0],
                "x_pos_cm": struct.unpack(">H", data[1:3])[0],
                "x_neg_cm": struct.unpack(">H", data[3:5])[0],
                "y_pos_cm": struct.unpack(">H", data[5:7])[0],
                "y_neg_cm": struct.unpack(">H", data[7:9])[0],
            }
    if control == 0x07 and command in (0x1A, 0x9A) and len(data) >= 1:
        return {"edge_mode": data[0]}
    if control == 0x07 and command in (0x11, 0x91):
        if not data:
            return {"labels_cleared": True}
        if len(data) >= 6:
            return {
                "tag_type": data[0],
                "tag_range_type": data[1],
                "tag_x_cm": struct.unpack(">H", data[2:4])[0],
                "tag_y_cm": struct.unpack(">H", data[4:6])[0],
            }
    if control == 0x80 and command in (0x00, 0x80) and len(data) >= 1:
        return {"human_presence_enabled": data[0] == 1}
    if control == 0x82 and command in (0x00, 0x80) and len(data) >= 1:
        return {"trajectory_enabled": data[0] == 1}
    if control == 0x86 and command in (0x0B, 0x8B) and len(data) >= 4:
        return {"realtime_people_interval_s": struct.unpack(">I", data[:4])[0]}
    if control == 0x86 and command in (0x0D, 0x8D) and len(data) >= 4:
        return {"accurate_people_interval_s": struct.unpack(">I", data[:4])[0]}
    if control == 0x86 and command in (0x0E, 0x8E) and len(data) >= 4:
        return {"track_distance_threshold_cm": struct.unpack(">I", data[:4])[0]}
    if control == 0x86 and command in (0x0F, 0x8F) and len(data) >= 4:
        return {"false_point_remove_s": struct.unpack(">I", data[:4])[0]}
    if control == 0x86 and command in (0x15, 0x95) and len(data) >= 4:
        return {"track_presence_time_s": struct.unpack(">I", data[:4])[0]}
    return None


def print_frames(label, frames):
    for frame in frames:
        parsed = parse_frame(frame)
        suffix = f" parsed={parsed}" if parsed is not None else ""
        print(
            f"{label}: {hex_with_spaces(frame['raw'])} "
            f"{'OK' if frame['checksum_ok'] else 'CHECKSUM_ERR'}{suffix}"
        )


def send_step(ser, parser, name, set_frame, query_frame, expected, delay_s):
    print(f"\n[{name}]")
    print(f"Set:   {hex_with_spaces(set_frame)}")
    print(f"Query: {hex_with_spaces(query_frame)}")

    ser.write(set_frame)
    ser.flush()
    print_frames("Set response", read_frames(ser, parser, delay_s))

    ser.write(query_frame)
    ser.flush()
    query_frames = read_frames(ser, parser, delay_s + 1.0)
    print_frames("Query response", query_frames)

    parsed_items = [
        parsed
        for parsed in (parse_frame(frame) for frame in query_frames if frame["checksum_ok"])
        if parsed
    ]
    if expected not in parsed_items:
        raise SystemExit(
            f"{name} verification failed: expected {expected}, got {parsed_items}"
        )
    print(f"Verified: {expected}")


def make_steps(args):
    angle_data = (
        pack_i16(round(args.angle_x_deg * 100))
        + pack_i16(round(args.angle_y_deg * 100))
        + pack_i16(round(args.angle_z_deg * 100))
    )
    range_data = (
        b"\x00"
        + pack_u16(args.x_pos_cm)
        + pack_u16(args.x_neg_cm)
        + pack_u16(args.y_pos_cm)
        + pack_u16(args.y_neg_cm)
    )
    bed_label_data = (
        bytes([args.bed_tag_type, args.bed_tag_range_type])
        + pack_u16(args.bed_label_x_cm)
        + pack_u16(args.bed_label_y_cm)
    )

    steps = [
        (
            "安装角度",
            build_frame(0x06, 0x01, angle_data),
            build_frame(0x06, 0x81, b"\x0F"),
            {
                "angle_x_deg": float(args.angle_x_deg),
                "angle_y_deg": float(args.angle_y_deg),
                "angle_z_deg": float(args.angle_z_deg),
            },
        ),
        (
            "安装高度",
            build_frame(0x06, 0x02, pack_u16(args.height_cm)),
            build_frame(0x06, 0x82, b"\x0F"),
            {"height_cm": args.height_cm},
        ),
    ]

    if not args.keep_labels:
        steps.append(
            (
                "清除全部标签",
                build_frame(0x07, 0x13, b"\xFF"),
                build_frame(0x07, 0x91, b"\x0F"),
                {"labels_cleared": True},
            )
        )

    if args.set_bed_label:
        steps.append(
            (
                "床铺标签",
                build_frame(0x07, 0x11, bed_label_data),
                build_frame(0x07, 0x91, b"\x0F"),
                None,
            )
        )

    steps.extend(
        [
        (
            "开启人体存在",
            build_frame(0x80, 0x00, b"\x01"),
            build_frame(0x80, 0x80, b"\x0F"),
            {"human_presence_enabled": True},
        ),
        (
            "开启轨迹跟踪",
            build_frame(0x82, 0x00, b"\x01"),
            build_frame(0x82, 0x80, b"\x0F"),
            {"trajectory_enabled": True},
        ),
        (
            "启用实时人数上报（间隔）",
            build_frame(0x86, 0x0B, pack_u32(args.realtime_people_interval_s)),
            build_frame(0x86, 0x8B, b"\x0F"),
            {"realtime_people_interval_s": args.realtime_people_interval_s},
        ),
        (
            "启用精准人数上报（间隔）",
            build_frame(0x86, 0x0D, pack_u32(args.accurate_people_interval_s)),
            build_frame(0x86, 0x8D, b"\x0F"),
            {"accurate_people_interval_s": args.accurate_people_interval_s},
        ),
        (
            "轨迹产生距离阈值",
            build_frame(0x86, 0x0E, pack_u32(args.track_distance_threshold_cm)),
            build_frame(0x86, 0x8E, b"\x0F"),
            {"track_distance_threshold_cm": args.track_distance_threshold_cm},
        ),
        (
            "误报点消除时长",
            build_frame(0x86, 0x0F, pack_u32(args.false_point_remove_s)),
            build_frame(0x86, 0x8F, b"\x0F"),
            {"false_point_remove_s": args.false_point_remove_s},
        ),
        (
            "存在轨迹时间",
            build_frame(0x86, 0x15, pack_u32(args.track_presence_time_s)),
            build_frame(0x86, 0x95, b"\x0F"),
            {"track_presence_time_s": args.track_presence_time_s},
        ),
        ]
    )
    if args.set_fixed_range:
        steps.extend(
            [
                (
                    "关闭自动范围使用",
                    build_frame(0x07, 0x0C, b"\x00"),
                    build_frame(0x07, 0x89, b"\x0F"),
                    None,
                ),
                (
                    "固定探测边界",
                    build_frame(0x07, 0x09, range_data),
                    build_frame(0x07, 0x89, b"\x0F"),
                    {
                        "range_mode": 0,
                        "x_pos_cm": args.x_pos_cm,
                        "x_neg_cm": args.x_neg_cm,
                        "y_pos_cm": args.y_pos_cm,
                        "y_neg_cm": args.y_neg_cm,
                    },
                ),
            ]
        )
    else:
        steps.extend(
            [
                (
                    "设置自动范围宿舍边界",
                    build_frame(0x07, 0x09, range_data),
                    build_frame(0x07, 0x89, b"\x0F"),
                    {
                        "range_mode": 0,
                        "x_pos_cm": args.x_pos_cm,
                        "x_neg_cm": args.x_neg_cm,
                        "y_pos_cm": args.y_pos_cm,
                        "y_neg_cm": args.y_neg_cm,
                    },
                ),
                (
                    "启用自动探测范围",
                    build_frame(0x07, 0x0C, b"\x01"),
                    build_frame(0x07, 0x89, b"\x0F"),
                    {"range_mode": 1},
                ),
                (
                    "开启自动探测范围限制（最后发送）",
                    build_frame(0x07, 0x08, b"\x01"),
                    build_frame(0x07, 0x89, b"\x0F"),
                    {"range_mode": 1},
                ),
            ]
        )
    return steps


def main():
    arg_parser = argparse.ArgumentParser(
        description=(
            "Configure Yunfan R60BMP1 for a 5m x 3.3m dorm bunk-bed scene "
            "using 90-degree installation at the ceiling center."
        )
    )
    arg_parser.add_argument("--port", default="COM3", help="Serial port, default COM3")
    arg_parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    arg_parser.add_argument("--angle-x-deg", type=float, default=0.0)
    arg_parser.add_argument("--angle-y-deg", type=float, default=0.0)
    arg_parser.add_argument("--angle-z-deg", type=float, default=90.0)
    arg_parser.add_argument("--height-cm", type=int, default=330)
    arg_parser.add_argument("--x-pos-cm", type=int, default=165)
    arg_parser.add_argument("--x-neg-cm", type=int, default=165)
    arg_parser.add_argument("--y-pos-cm", type=int, default=250)
    arg_parser.add_argument("--y-neg-cm", type=int, default=250)
    range_group = arg_parser.add_mutually_exclusive_group()
    range_group.add_argument(
        "--set-fixed-range",
        dest="set_fixed_range",
        action="store_true",
        help=(
            "Use the fixed room boundary instead of automatic range."
        ),
    )
    range_group.add_argument(
        "--keep-auto-range",
        dest="set_fixed_range",
        action="store_false",
        help="Use the radar's automatic 90-degree detection range (default).",
    )
    arg_parser.set_defaults(set_fixed_range=False)
    arg_parser.add_argument("--realtime-people-interval-s", type=int, default=1)
    arg_parser.add_argument("--accurate-people-interval-s", type=int, default=60)
    arg_parser.add_argument("--track-distance-threshold-cm", type=int, default=50)
    arg_parser.add_argument("--false-point-remove-s", type=int, default=30)
    arg_parser.add_argument("--track-presence-time-s", type=int, default=2)
    arg_parser.add_argument(
        "--keep-labels",
        action="store_true",
        help=(
            "Keep labels generated by the radar. By default all labels are "
            "cleared after setting the 90-degree angle."
        ),
    )
    arg_parser.add_argument(
        "--set-bed-label",
        action="store_true",
        help=(
            "Also send 0x07/0x11 bed label settings. Use only after bed-zone "
            "coordinates are calibrated on site."
        ),
    )
    arg_parser.add_argument(
        "--bed-tag-type",
        type=lambda value: int(value, 0),
        default=0x0A,
        help="Bed tag type/index, default 0x0A. Valid bed range is 0x0A-0x0E.",
    )
    arg_parser.add_argument(
        "--bed-tag-range-type",
        type=int,
        default=1,
        choices=(0, 1),
        help="0=circle, 1=rectangle. Default 1.",
    )
    arg_parser.add_argument(
        "--bed-label-x-cm",
        type=int,
        default=90,
        help="Bed label X parameter: radius for circle, width for rectangle.",
    )
    arg_parser.add_argument(
        "--bed-label-y-cm",
        type=int,
        default=200,
        help="Bed label Y parameter: unused for circle, length for rectangle.",
    )
    arg_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print frames; do not open serial port.",
    )
    arg_parser.add_argument(
        "--delay-s",
        type=float,
        default=1.0,
        help="Seconds to wait for each set/query response.",
    )
    args = arg_parser.parse_args()
    if not 0x0A <= args.bed_tag_type <= 0x0E:
        raise SystemExit("--bed-tag-type must be in the bed range 0x0A-0x0E")

    steps = make_steps(args)

    print("Dorm bunk-bed ceiling-installation radar configuration")
    print(f"Port: {args.port}, baud: {args.baud}")
    print(
        "Room boundary: "
        f"X+={args.x_pos_cm}cm, X-={args.x_neg_cm}cm, "
        f"Y+={args.y_pos_cm}cm, Y-={args.y_neg_cm}cm "
        f"({'fixed mode' if args.set_fixed_range else 'automatic range limit'})"
    )
    print(
        "Mounting: "
        f"X={args.angle_x_deg:.2f}deg, "
        f"Y={args.angle_y_deg:.2f}deg, "
        f"Z={args.angle_z_deg:.2f}deg, "
        f"height={args.height_cm}cm"
    )
    print(f"Labels: {'kept' if args.keep_labels else 'all cleared'}")
    print(
        "Interference-edge setting is intentionally skipped: 0x07/0x1A is "
        "documented only for 30-degree angled installation."
    )
    if args.set_bed_label:
        range_name = "rectangle" if args.bed_tag_range_type == 1 else "circle"
        print(
            "Bed label: "
            f"type=0x{args.bed_tag_type:02X}, range={range_name}, "
            f"X={args.bed_label_x_cm}cm, Y={args.bed_label_y_cm}cm"
        )

    if args.dry_run:
        for name, set_frame, query_frame, expected in steps:
            print(f"\n[{name}]")
            print(f"Set:   {hex_with_spaces(set_frame)}")
            print(f"Query: {hex_with_spaces(query_frame)}")
            if expected is not None:
                print(f"Expect: {expected}")
        return

    parser = FrameParser()
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        for name, set_frame, query_frame, expected in steps:
            if expected is None:
                print(f"\n[{name}]")
                print(f"Set:   {hex_with_spaces(set_frame)}")
                print(f"Query: {hex_with_spaces(query_frame)}")
                ser.write(set_frame)
                ser.flush()
                print_frames("Set response", read_frames(ser, parser, args.delay_s))
                ser.write(query_frame)
                ser.flush()
                print_frames(
                    "Query response", read_frames(ser, parser, args.delay_s + 1.0)
                )
            else:
                send_step(
                    ser,
                    parser,
                    name,
                    set_frame,
                    query_frame,
                    expected,
                    args.delay_s,
                )

    print("\nAll requested settings were sent and verified.")


if __name__ == "__main__":
    main()
