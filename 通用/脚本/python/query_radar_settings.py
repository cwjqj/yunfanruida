"""查询云帆瑞达 R60BMP1 雷达当前参数。

协议依据：``通用/资料/云帆瑞达.pdf``（R60BMP1 用户手册 v1.5），
PDF 第 6～19 页的串口协议、查询命令和响应数据定义。
"""

import argparse
import sys
import codecs
if sys.platform == 'win32':
    try:
        sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    except AttributeError:
        pass

import serial
import struct
import time

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
QUERY_TIMEOUT = 2.0

FRAME_HEAD = b'\x53\x59'
FRAME_TAIL = b'\x54\x43'
MAX_DATA_LENGTH = 4096


def calc_checksum(frame_bytes):
    return sum(frame_bytes) & 0xFF


def build_query(control, command, data=b'\x0F'):
    length = len(data)
    frame = bytearray()
    frame.extend(b'\x53\x59')
    frame.append(control)
    frame.append(command)
    frame.extend(struct.pack('>H', length))
    frame.extend(data)
    checksum = calc_checksum(frame)
    frame.append(checksum)
    frame.extend(b'\x54\x43')
    return bytes(frame)


def parse_int16_be(data):
    val = struct.unpack('>H', data)[0]
    if val & 0x8000:
        val -= 0x10000
    return val


def parse_sign_magnitude16_be(data):
    """解析协议所述“最高位为符号位”的 16 位位置数据。"""
    val = struct.unpack('>H', data)[0]
    if val & 0x8000:
        return -(val & 0x7FFF)
    return val


def parse_uint16_be(data):
    return struct.unpack('>H', data)[0]


def parse_uint32_be(data):
    return struct.unpack('>I', data)[0]


def hex_with_spaces(data):
    return ' '.join(f'{b:02X}' for b in data)


def tag_type_name(tag_type):
    if 0 <= tag_type <= 4:
        return f"门{tag_type}"
    if 5 <= tag_type <= 9:
        return f"沙发{tag_type}"
    if 10 <= tag_type <= 14:
        return f"床{tag_type}"
    if 15 <= tag_type <= 19:
        return f"干扰源{tag_type}"
    if tag_type == 20:
        return "卫生间门20"
    if 21 <= tag_type <= 30:
        return f"保留标签{tag_type}"
    return f"未知标签{tag_type}"


def parse_label_records(data):
    """解析 0x07/0x91 返回的 n 个 10 字节有效标签。"""
    if not data:
        return "标签: 未设置"
    if len(data) % 10:
        raise ValueError(f"标签数据长度应为 10 的倍数，实际为 {len(data)}")

    labels = []
    for offset in range(0, len(data), 10):
        tag_type = data[offset]
        range_value = data[offset + 1]
        center_x = parse_sign_magnitude16_be(data[offset + 2:offset + 4])
        center_y = parse_sign_magnitude16_be(data[offset + 4:offset + 6])
        width = parse_uint16_be(data[offset + 6:offset + 8])
        height = parse_uint16_be(data[offset + 8:offset + 10])
        if range_value == 0:
            shape = f"圆形, 半径={width}cm"
        elif range_value == 1:
            shape = f"矩形, 宽={width}cm, 高={height}cm"
        else:
            shape = f"未知范围类型({range_value}), X={width}cm, Y={height}cm"
        labels.append(
            f"{tag_type_name(tag_type)}: {shape}, 中心=({center_x},{center_y})cm"
        )
    return " | ".join(labels)


def parse_configured_range(data):
    """解析 0x07/0x97 返回的门信息和探测范围坐标点。"""
    if len(data) < 9 or (len(data) - 9) % 4:
        raise ValueError(f"配置文件探测范围数据长度无效: {len(data)}")
    center_x = parse_sign_magnitude16_be(data[0:2])
    center_y = parse_sign_magnitude16_be(data[2:4])
    width = parse_uint16_be(data[4:6])
    height = parse_uint16_be(data[6:8])
    range_value = data[8]
    range_type = {0: "圆形", 1: "矩形"}.get(
        range_value, f"未知({range_value})"
    )


def validate_response_data(control, command, data):
    """按 PDF 的回复结构检查长度，防止把截断帧记为查询成功。"""
    exact_lengths = {
        (0x05, 0x81): 1,
        (0x06, 0x81): 6,
        (0x06, 0x82): 2,
        (0x80, 0x80): 1,
        (0x80, 0x81): 1,
        (0x80, 0x82): 1,
        (0x80, 0x83): 1,
        (0x07, 0x9A): 1,
        (0x82, 0x80): 1,
        (0x86, 0x8A): 2,
        (0x86, 0x8B): 4,
        (0x86, 0x8C): 2,
        (0x86, 0x8D): 4,
        (0x86, 0x8E): 4,
        (0x86, 0x8F): 4,
        (0x86, 0x95): 4,
    }
    expected = exact_lengths.get((control, command))
    if expected is not None and len(data) != expected:
        return False, f"应为 {expected}B，实际为 {len(data)}B"
    if (control, command) == (0x02, 0xA4) and not data:
        return False, "固件版本数据为空"
    if (control, command) == (0x07, 0x89):
        if not data or data[0] not in (0, 1):
            return False, "探测范围缺少有效模式字节"
        if data[0] == 0 and len(data) != 9:
            return False, f"手动探测范围应为 9B，实际为 {len(data)}B"
        if data[0] == 1 and (len(data) - 1) % 4:
            return False, "自动探测范围坐标长度不是 4B 的倍数"
    if (control, command) == (0x07, 0x91) and len(data) % 10:
        return False, "标签数据长度不是 10B 的倍数"
    if (control, command) == (0x07, 0x97):
        if len(data) < 9 or (len(data) - 9) % 4:
            return False, "配置文件探测范围长度不符合 9B+n*4B"
    if (control, command) == (0x82, 0x82) and len(data) % 11:
        return False, "轨迹数据长度不是 11B 的倍数"
    return True, ""
    points = []
    for offset in range(9, len(data), 4):
        x = parse_sign_magnitude16_be(data[offset:offset + 2])
        y = parse_sign_magnitude16_be(data[offset + 2:offset + 4])
        points.append(f"({x},{y})")
    point_text = ", ".join(points) if points else "无"
    return (
        f"配置文件探测范围: 门中心=({center_x},{center_y})cm, "
        f"宽={width}cm, 高={height}cm, 类型={range_type}, 坐标点={point_text}"
    )


# 控制字映射
CONTROL_WORD_MAP = {
    0x01: "心跳包", 0x02: "产品信息", 0x03: "OTA升级",
    0x05: "工作状态", 0x06: "安装方式", 0x07: "人数统计配置",
    0x80: "人体存在", 0x82: "轨迹跟踪", 0x86: "人数统计",
}


def parse_frame_data(control, command, data):
    """解析帧数据"""
    try:
        if control == 0x02 and command == 0xA4:
            version = data.rstrip(b'\x00').decode('ascii', errors='replace')
            return f"固件版本: {version} (原始: {hex_with_spaces(data)})"
        elif control == 0x05:
            if command == 0x01:
                return "初始化完成(主动上报)"
            elif command == 0x81:
                return f"初始化状态: {'已完成' if data[0]==0x01 else '未完成'}"
        elif control == 0x06:
            if command in (0x01, 0x81):
                x = parse_int16_be(data[0:2]) / 100.0
                y = parse_int16_be(data[2:4]) / 100.0
                z = parse_int16_be(data[4:6]) / 100.0
                return f"X轴角度: {x:.2f}°, Y轴角度: {y:.2f}°, Z轴下倾角: {z:.2f}°"
            elif command in (0x02, 0x82):
                return f"安装高度: {parse_uint16_be(data[0:2])} cm"
        elif control == 0x07:
            if command == 0x08:
                return f"自动探测范围限制: {'开启' if data[0]==0x01 else '关闭'}"
            elif command in (0x09, 0x89):
                if data[0] == 1:
                    points = []
                    for offset in range(1, len(data), 4):
                        x = parse_sign_magnitude16_be(data[offset:offset + 2])
                        y = parse_sign_magnitude16_be(data[offset + 2:offset + 4])
                        points.append(f"({x},{y})")
                    point_text = ", ".join(points) if points else "未返回坐标点"
                    return f"探测范围(自动): {point_text}"
                x_pos = parse_uint16_be(data[1:3])
                x_neg = parse_uint16_be(data[3:5])
                y_pos = parse_uint16_be(data[5:7])
                y_neg = parse_uint16_be(data[7:9])
                return (
                    f"探测范围(手动): X+={x_pos}cm, X-={x_neg}cm, "
                    f"Y+={y_pos}cm, Y-={y_neg}cm"
                )
            elif command in (0x11, 0x91):
                return parse_label_records(data)
            elif command in (0x17, 0x97):
                return parse_configured_range(data)
            elif command in (0x1A, 0x9A):
                edge_map = {1: "左侧0.5m边缘,前方4m", 2: "右侧0.5m边缘,前方4m",
                            3: "酒店走廊,左右各0.75m", 4: "无边界设置"}
                return f"干扰边设置: {edge_map.get(data[0], f'未知({data[0]})')}"
        elif control == 0x80:
            if command in (0x00, 0x80):
                return f"人体存在功能: {'开启' if data[0]==0x01 else '关闭'}"
            elif command in (0x01, 0x81):
                return f"人体存在状态: {'有人' if data[0]==0x01 else '无人'}"
            elif command in (0x02, 0x82):
                m = {0: "无运动", 1: "静止", 2: "活跃"}
                return f"运动状态: {m.get(data[0], f'未知({data[0]})')}"
            elif command in (0x03, 0x83):
                return f"体动参数: {data[0]} (0-100)"
        elif control == 0x82:
            if command in (0x00, 0x80):
                return f"轨迹跟踪功能: {'开启' if data[0]==0x01 else '关闭'}"
            elif command in (0x02, 0x82):
                targets = []
                offset = 0
                while offset + 11 <= len(data):
                    idx = data[offset]
                    size = data[offset+1]
                    feature = data[offset+2]
                    x = parse_sign_magnitude16_be(data[offset+3:offset+5])
                    y = parse_sign_magnitude16_be(data[offset+5:offset+7])
                    h = parse_uint16_be(data[offset+7:offset+9])
                    spd = parse_sign_magnitude16_be(data[offset+9:offset+11])
                    targets.append(
                        f"目标{idx}: 大小={size}, 特征=0x{feature:02X}, "
                        f"位置=({x},{y})cm, 高度={h}cm, 速度={spd}cm/s"
                    )
                    offset += 11
                remainder = len(data) - offset
                suffix = f" | 未解析尾部={remainder}B" if remainder else ""
                return (" | ".join(targets) if targets else "无轨迹目标") + suffix
        elif control == 0x86:
            if command in (0x0A, 0x8A):
                return f"实时人数: 最小={data[0]}, 最大={data[1]}"
            elif command in (0x0B, 0x8B):
                return f"实时人数上报间隔: {parse_uint32_be(data[0:4])} 秒"
            elif command in (0x0C, 0x8C):
                return f"精准人数: 最小={data[0]}, 最大={data[1]}"
            elif command in (0x0D, 0x8D):
                return f"精准人数上报间隔: {parse_uint32_be(data[0:4])} 秒"
            elif command in (0x0E, 0x8E):
                return f"轨迹产生距离: {parse_uint32_be(data[0:4])} cm"
            elif command in (0x0F, 0x8F):
                return f"误报点消除时长: {parse_uint32_be(data[0:4])} 秒"
            elif command in (0x15, 0x95):
                return f"存在轨迹时间: {parse_uint32_be(data[0:4])} 秒"
        return f"原始数据: {hex_with_spaces(data)}"
    except Exception as e:
        return f"解析异常: {e}, 原始: {hex_with_spaces(data)}"


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, raw_bytes):
        self.buffer.extend(raw_bytes)
        frames = []
        while True:
            frame = self._try_parse_one()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _try_parse_one(self):
        while True:
            head_idx = self.buffer.find(FRAME_HEAD)
            if head_idx == -1:
                # 保留末尾可能是下一帧帧头首字节的 0x53。
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\x53' else b''
                return None
            if head_idx > 0:
                del self.buffer[:head_idx]
            if len(self.buffer) < 9:
                return None

            data_len = struct.unpack('>H', self.buffer[4:6])[0]
            if data_len > MAX_DATA_LENGTH:
                del self.buffer[0]
                continue
            total_len = 9 + data_len
            if len(self.buffer) < total_len:
                return None
            if bytes(self.buffer[total_len - 2:total_len]) != FRAME_TAIL:
                del self.buffer[0]
                continue

            frame_bytes = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]
            control = frame_bytes[2]
            command = frame_bytes[3]
            data = frame_bytes[6:6 + data_len]
            checksum = frame_bytes[6 + data_len]
            valid = calc_checksum(frame_bytes[:6 + data_len]) == checksum
            return {
                'control': control,
                'command': command,
                'data': data,
                'data_hex': hex_with_spaces(data),
                'checksum_valid': valid,
                'description': parse_frame_data(control, command, data),
                'raw_hex': hex_with_spaces(frame_bytes),
            }


QUERIES = [
    ("固件版本",           0x02, 0xA4, b'\x0F'),
    ("初始化状态",         0x05, 0x81, b'\x0F'),
    ("安装角度",           0x06, 0x81, b'\x0F'),
    ("安装高度",           0x06, 0x82, b'\x0F'),
    ("人体存在开关",       0x80, 0x80, b'\x0F'),
    ("人体存在状态",       0x80, 0x81, b'\x0F'),
    ("运动信息",           0x80, 0x82, b'\x0F'),
    ("体动参数",           0x80, 0x83, b'\x0F'),
    ("雷达探测范围",       0x07, 0x89, b'\x0F'),
    ("标签信息",           0x07, 0x91, b'\x0F'),
    ("配置文件探测范围",    0x07, 0x97, b'\x0F'),
    ("干扰边设置",         0x07, 0x9A, b'\x0F'),
    ("轨迹跟踪开关",       0x82, 0x80, b'\x0F'),
    ("轨迹信息",           0x82, 0x82, b'\x0F'),
    ("实时人数",           0x86, 0x8A, b'\x0F'),
    ("实时人数上报间隔",    0x86, 0x8B, b'\x0F'),
    ("精准人数",           0x86, 0x8C, b'\x0F'),
    ("精准人数上报间隔",    0x86, 0x8D, b'\x0F'),
    ("轨迹产生米数",       0x86, 0x8E, b'\x0F'),
    ("误报点消除时长",      0x86, 0x8F, b'\x0F'),
    ("存在轨迹时间",       0x86, 0x95, b'\x0F'),
]


def main():
    arg_parser = argparse.ArgumentParser(
        description="查询云帆瑞达 R60BMP1 当前内部参数"
    )
    arg_parser.add_argument("--port", default=SERIAL_PORT, help="串口，默认 COM3")
    arg_parser.add_argument("--baud", type=int, default=BAUD_RATE, help="波特率")
    arg_parser.add_argument(
        "--timeout",
        type=float,
        default=QUERY_TIMEOUT,
        help="每项查询超时秒数",
    )
    args = arg_parser.parse_args()

    print("=" * 60)
    print("  云帆瑞达 R60BMP1 雷达设置信息查询")
    print("=" * 60)
    print(f"  串口: {args.port}")
    print(f"  波特率: {args.baud}")
    print(f"  查询超时: {args.timeout}s")
    print(f"  查询项: {len(QUERIES)} 项")
    print("=" * 60)

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )
        print(f"\n串口 {args.port} 已打开")
    except serial.SerialException as e:
        print(f"\n无法打开串口 {args.port}: {e}")
        return 1

    results = {}

    try:
        for name, ctrl, cmd, data in QUERIES:
            query_frame = build_query(ctrl, cmd, data)
            hex_str = ' '.join(f'{b:02X}' for b in query_frame)
            print(f"\n>>> 查询 [{name}]: {hex_str}")

            # 清空缓冲
            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)
            parser = FrameParser()

            ser.write(query_frame)

            start = time.time()
            responded = False
            while time.time() - start < args.timeout:
                if ser.in_waiting > 0:
                    raw = ser.read(ser.in_waiting)
                    frames = parser.feed(raw)
                    for frame in frames:
                        cw_name = CONTROL_WORD_MAP.get(frame['control'], f"0x{frame['control']:02X}")
                        print(f"    [{cw_name}] cmd=0x{frame['command']:02X} data={frame['data_hex']} {'OK' if frame['checksum_valid'] else 'CHK_ERR'}")
                        print(f"    => {frame['description']}")
                        if (
                            frame['checksum_valid']
                            and frame['control'] == ctrl
                            and frame['command'] == cmd
                        ):
                            data_valid, reason = validate_response_data(
                                frame['control'], frame['command'], frame['data']
                            )
                            if data_valid:
                                results[name] = frame
                                responded = True
                            else:
                                print(f"    [长度错误] {reason}，继续等待有效回复")
                if responded:
                    time.sleep(0.3)
                    if ser.in_waiting > 0:
                        raw = ser.read(ser.in_waiting)
                        for frame in parser.feed(raw):
                            cw_name = CONTROL_WORD_MAP.get(frame['control'], f"0x{frame['control']:02X}")
                            print(f"    [{cw_name}] cmd=0x{frame['command']:02X} data={frame['data_hex']} {'OK' if frame['checksum_valid'] else 'CHK_ERR'}")
                            print(f"    => {frame['description']}")
                    break
                time.sleep(0.05)

            if not responded:
                print(f"    [超时] 无响应")
            time.sleep(0.3)

        # 汇总
        print("\n" + "=" * 60)
        print("  查询结果汇总")
        print("=" * 60)
        for name, ctrl, cmd, data in QUERIES:
            if name in results:
                print(f"  {name:14s}: {results[name]['description']}")
            else:
                print(f"  {name:14s}: [无响应]")

    except KeyboardInterrupt:
        print("\n\n查询中断")
    finally:
        ser.close()
        print("\n串口已关闭")
    return 0


if __name__ == "__main__":
    sys.exit(main())
