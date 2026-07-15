"""
云帆瑞达 60G 毫米波雷达串口数据解析程序
协议参考: 串口通信协议详解.md
"""

import sys
import codecs
if sys.platform == 'win32':
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')

import serial
import struct
import time
import threading
from datetime import datetime
from collections import deque
from pathlib import Path


def hex_with_spaces(data: bytes) -> str:
    """将字节转换为空格分隔的十六进制字符串"""
    return ' '.join(f'{b:02X}' for b in data)

# ======================== 常量定义 ========================

FRAME_HEAD = b'\x53\x59'
FRAME_TAIL = b'\x54\x43'
HEAD_LEN = 2
TAIL_LEN = 2
FRAME_MIN_LEN = HEAD_LEN + 1 + 1 + 2 + 1 + TAIL_LEN  # 7 + 头尾 = 最小帧长度(无数据时)

# 控制字映射
CONTROL_WORD_MAP = {
    0x01: "心跳包",
    0x02: "产品信息",
    0x03: "OTA升级",
    0x05: "工作状态",
    0x06: "安装方式",
    0x07: "人数统计配置",
    0x80: "人体存在",
    0x82: "轨迹跟踪",
    0x86: "人数统计",
}

# ======================== 有符号 16 位解析 ========================

def parse_int16_be(data: bytes) -> int:
    """解析大端序有符号 16 位整数"""
    val = struct.unpack('>H', data)[0]
    if val & 0x8000:
        val -= 0x10000
    return val

def parse_uint16_be(data: bytes) -> int:
    """解析大端序无符号 16 位整数"""
    return struct.unpack('>H', data)[0]

def parse_uint32_be(data: bytes) -> int:
    """解析大端序无符号 32 位整数"""
    return struct.unpack('>I', data)[0]


# ======================== 各控制字解析函数 ========================

def parse_heartbeat(data: bytes, cmd: int) -> str:
    """心跳包 (0x01)"""
    return f"数据: 0x{data[0]:02X}"

def parse_product_info(data: bytes, cmd: int) -> str:
    """产品信息 (0x02)"""
    if cmd == 0xA4:
        version = data.decode('ascii', errors='replace') if data else "无"
        return f"固件版本: {version}"
    return f"原始数据: {data.hex()}"

def parse_work_status(data: bytes, cmd: int) -> str:
    """工作状态 (0x05)"""
    if cmd == 0x01:
        return "状态: 初始化完成(主动上报)"
    elif cmd == 0x81:
        status = "已完成" if data[0] == 0x01 else "未完成"
        return f"初始化状态: {status}"
    return f"原始数据: {data.hex()}"

def parse_install_info(data: bytes, cmd: int) -> str:
    """安装方式 (0x06)"""
    if cmd == 0x01:
        x = parse_int16_be(data[0:2]) / 100.0
        y = parse_int16_be(data[2:4]) / 100.0
        z = parse_int16_be(data[4:6]) / 100.0
        return f"X轴角度: {x:.2f}° | Y轴角度: {y:.2f}° | Z轴下倾角: {z:.2f}°"
    elif cmd == 0x02:
        height = parse_uint16_be(data[0:2])
        return f"安装高度: {height} cm"
    elif cmd == 0x04:
        yaw = parse_int16_be(data[0:2]) / 100.0
        pitch = parse_int16_be(data[2:4]) / 100.0
        yaw_std = parse_uint16_be(data[4:6]) / 100.0
        pitch_std = parse_uint16_be(data[6:8]) / 100.0
        return (f"水平角(偏航): {yaw:.2f}° | 俯仰角: {pitch:.2f}° | "
                f"水平标准差: {yaw_std:.2f}° | 俯仰标准差: {pitch_std:.2f}°")
    elif cmd == 0x05:
        status_map = {0: "正常", 1: "无传感器", 2: "测量角度与预设差异过大(>±5°)"}
        status = status_map.get(data[0], f"未知({data[0]})")
        return f"陀螺仪异常状态: {status}"
    return f"原始数据: {data.hex()}"

def parse_count_config(data: bytes, cmd: int) -> str:
    """人数统计配置 (0x07)"""
    if cmd == 0x08:
        status = "开启自动探测" if data[0] == 0x01 else "关闭自动探测"
        extra = ""
        if data[0] == 0x00 and len(data) > 1:
            # 关闭时返回探测点坐标
            pts = []
            for i in range(1, len(data) - 1, 4):
                if i + 3 < len(data):
                    x = parse_int16_be(data[i:i+2])
                    y = parse_int16_be(data[i+2:i+4])
                    pts.append(f"({x},{y})")
            extra = f" | 探测点: {', '.join(pts)}"
        return f"自动探测范围限制: {status}{extra}"
    elif cmd == 0x09:
        x_pos = parse_uint16_be(data[1:3])
        x_neg = parse_uint16_be(data[3:5])
        y_pos = parse_uint16_be(data[5:7])
        y_neg = parse_uint16_be(data[7:9])
        return (f"探测范围 — X+: {x_pos}cm, X-: {x_neg}cm, "
                f"Y+: {y_pos}cm, Y-: {y_neg}cm")
    elif cmd == 0x0C:
        status = "使用" if data[0] == 0x01 else "不使用"
        return f"自动探测范围: {status}"
    elif cmd == 0x11:
        tag_type = data[0]
        range_type = "圆形" if data[1] == 0 else "矩形"
        x = parse_uint16_be(data[2:4])
        y = parse_uint16_be(data[4:6])
        tag_names = {
            range(0, 5): "门", range(5, 10): "沙发", range(10, 15): "床",
            range(15, 20): "干扰源", range(20, 21): "卫生间门",
            range(21, 31): "保留"
        }
        tag_name = "未知"
        for r, name in tag_names.items():
            if tag_type in r:
                tag_name = name
                break
        detail = ""
        if range_type == "圆形":
            detail = f"半径: {x}cm"
        else:
            detail = f"宽: {x}cm, 高: {y}cm"
        return f"标签: {tag_name} | 范围: {range_type} | {detail}"
    elif cmd == 0x12:
        return f"标签验证: {data.hex()}"
    elif cmd == 0x13:
        return "标签清除"
    elif cmd == 0x14 or cmd == 0x15:
        return f"标签信息上报: {data.hex()}"
    elif cmd == 0x17:
        door_x = parse_int16_be(data[0:2])
        door_y = parse_int16_be(data[2:4])
        door_w = parse_uint16_be(data[4:6])
        door_h = parse_uint16_be(data[6:8])
        range_t = "圆形" if data[8] == 0 else "矩形"
        pts = []
        for i in range(9, len(data) - 1, 4):
            if i + 3 < len(data):
                px = parse_int16_be(data[i:i+2])
                py = parse_int16_be(data[i+2:i+4])
                pts.append(f"({px},{py})")
        return (f"探测范围配置 — 门中心: ({door_x},{door_y})cm, "
                f"宽: {door_w}cm, 高: {door_h}cm, 类型: {range_t} | "
                f"坐标点: {', '.join(pts)}")
    elif cmd == 0x1A:
        edge_map = {
            1: "左侧0.5m边缘, 前方探测距离4m",
            2: "右侧0.5m边缘, 前方探测距离4m",
            3: "酒店走廊, 左右各0.75m",
            4: "无边界设置",
        }
        return f"干扰边设置: {edge_map.get(data[0], f'未知({data[0]})')}"
    return f"原始数据: {data.hex()}"

def parse_human_presence(data: bytes, cmd: int) -> str:
    """人体存在 (0x80)"""
    if cmd == 0x00:
        status = "开启" if data[0] == 0x01 else "关闭"
        return f"人体存在功能: {status}"
    elif cmd == 0x01:
        status = "有人" if data[0] == 0x01 else "无人"
        return f"人体存在状态: {status}"
    elif cmd == 0x02:
        motion_map = {0: "无运动", 1: "静止", 2: "活跃"}
        return f"运动状态: {motion_map.get(data[0], f'未知({data[0]})')}"
    elif cmd == 0x03:
        return f"体动参数: {data[0]} (0-100)"
    elif cmd == 0x80:
        return "查询: 人体存在开关"
    elif cmd == 0x81:
        return "查询: 存在信息"
    elif cmd == 0x82:
        return "查询: 运动信息"
    elif cmd == 0x83:
        return "查询: 体动参数"
    return f"原始数据: {data.hex()}"

def parse_trajectory(data: bytes, cmd: int) -> str:
    """轨迹跟踪 (0x82)"""
    if cmd == 0x00:
        status = "开启" if data[0] == 0x01 else "关闭"
        return f"轨迹跟踪功能: {status}"
    elif cmd == 0x02:
        # 每个目标 12 字节: 索引(1)+大小(1)+特征(1)+X(2)+Y(2)+高度(2)+速度(2)+保留(1)=12
        targets = []
        offset = 0
        while offset + 12 <= len(data):
            idx = data[offset]
            size = data[offset + 1]
            feature = data[offset + 2]
            x = parse_int16_be(data[offset + 3:offset + 5])
            y = parse_int16_be(data[offset + 5:offset + 7])
            height = parse_uint16_be(data[offset + 7:offset + 9])
            speed = parse_int16_be(data[offset + 9:offset + 11])
            targets.append(
                f"目标{idx}: 大小={size}, 特征=0x{feature:02X}, "
                f"位置=({x},{y})cm, 高度={height}cm, 速度={speed}cm/s"
            )
            offset += 12
        if not targets:
            return "无轨迹目标"
        return " | ".join(targets)
    elif cmd == 0x82:
        return "查询: 轨迹信息"
    return f"原始数据: {data.hex()}"

def parse_people_count(data: bytes, cmd: int) -> str:
    """人数统计 (0x86)"""
    if cmd == 0x0A:
        return f"实时人数: 最小={data[0]}, 最大={data[1]}"
    elif cmd == 0x0B:
        interval = parse_uint32_be(data[0:4])
        return f"实时人数上报间隔: {interval} 秒"
    elif cmd == 0x0C:
        return f"精准人数: 最小={data[0]}, 最大={data[1]}"
    elif cmd == 0x0D:
        interval = parse_uint32_be(data[0:4])
        return f"精准人数上报间隔: {interval} 秒"
    elif cmd == 0x0E:
        distance = parse_uint32_be(data[0:4])
        return f"轨迹产生米数阈值: {distance} cm"
    elif cmd == 0x0F:
        duration = parse_uint32_be(data[0:4])
        return f"误报点消除时长: {duration} 秒"
    elif cmd == 0x11:
        return "清除人数信息"
    elif cmd == 0x14:
        direction = "进" if data[0] == 0x00 else "出"
        return f"进出门提示: {direction}"
    elif cmd == 0x15:
        duration = parse_uint32_be(data[0:4])
        return f"存在轨迹时间: {duration} 秒"
    elif cmd == 0x16:
        action = "远离" if data[0] == 0x00 else "靠近"
        tag_idx = data[1]
        return f"靠近/远离提示: {action}, 标签索引={tag_idx}"
    elif 0x8A <= cmd <= 0x95:
        return f"参数查询回复: {data.hex()}"
    return f"原始数据: {data.hex()}"

def parse_ota(data: bytes, cmd: int) -> str:
    """OTA 升级 (0x03)"""
    if cmd == 0x01:
        if len(data) == 4:
            size = parse_uint32_be(data[0:4])
            return f"OTA固件包大小: {size} 字节"
        return f"OTA每帧传输大小: {parse_uint32_be(data[0:4])} 字节"
    elif cmd == 0x02:
        if len(data) >= 5:
            offset = parse_uint32_be(data[0:4])
            return f"OTA数据包: 偏移={offset}, 数据长度={len(data)-4} 字节"
        status = "成功" if data[0] == 0x01 else "失败"
        return f"OTA接收状态: {status}"
    elif cmd == 0x03:
        if data[0] == 0x01:
            return "OTA升级完成"
        elif data[0] == 0x00:
            return "OTA升级未完成"
        return "OTA升级确认"
    return f"原始数据: {data.hex()}"


# 控制字 → 解析函数映射
PARSER_MAP = {
    0x01: parse_heartbeat,
    0x02: parse_product_info,
    0x03: parse_ota,
    0x05: parse_work_status,
    0x06: parse_install_info,
    0x07: parse_count_config,
    0x80: parse_human_presence,
    0x82: parse_trajectory,
    0x86: parse_people_count,
}


# ======================== 帧解析核心类 ========================

class ParsedFrame:
    """解析后的帧数据结构"""
    def __init__(self):
        self.timestamp: str = ""
        self.raw_hex: str = ""
        self.control_word: int = 0
        self.control_name: str = ""
        self.command_word: int = 0
        self.data_len: int = 0
        self.data_hex: str = ""
        self.checksum: int = 0
        self.checksum_valid: bool = False
        self.description: str = ""

    def to_dict(self) -> dict:
        return {
            "timestamp": self.timestamp,
            "raw_hex": self.raw_hex,
            "control_word": f"0x{self.control_word:02X}",
            "control_name": self.control_name,
            "command_word": f"0x{self.command_word:02X}",
            "data_len": self.data_len,
            "data_hex": self.data_hex,
            "checksum": f"0x{self.checksum:02X}",
            "checksum_valid": "✓" if self.checksum_valid else "✗",
            "description": self.description,
        }


class FrameParser:
    """雷达串口帧解析器"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, raw_bytes):
        """输入原始字节，返回解析到的完整帧列表"""
        self.buffer.extend(raw_bytes)
        frames = []
        while True:
            frame = self._try_parse_one()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _try_parse_one(self):
        """尝试从缓冲区解析一帧"""
        # 查找帧头
        head_idx = self.buffer.find(FRAME_HEAD)
        if head_idx == -1:
            self.buffer.clear()
            return None

        # 丢弃帧头前的无效数据
        if head_idx > 0:
            self.buffer = self.buffer[head_idx:]

        # 至少需要 8 字节才能读到长度字段: 头(2)+控制(1)+命令(1)+长度(2)+校验(1)+尾(2)
        if len(self.buffer) < 7:
            return None

        # 读取长度
        data_len = struct.unpack('>H', self.buffer[4:6])[0]
        total_len = HEAD_LEN + 1 + 1 + 2 + data_len + 1 + TAIL_LEN  # 头+控制+命令+长度+数据+校验+尾

        if len(self.buffer) < total_len:
            return None

        # 提取完整帧
        frame_bytes = self.buffer[:total_len]
        self.buffer = self.buffer[total_len:]

        return self._parse_frame(frame_bytes)

    def _parse_frame(self, frame: bytes) -> ParsedFrame:
        """解析完整帧"""
        result = ParsedFrame()
        result.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        result.raw_hex = hex_with_spaces(frame)

        control = frame[2]
        command = frame[3]
        data_len = struct.unpack('>H', frame[4:6])[0]
        data = frame[6:6 + data_len]
        checksum = frame[6 + data_len]
        tail = frame[6 + data_len + 1:6 + data_len + 3]

        result.control_word = control
        result.control_name = CONTROL_WORD_MAP.get(control, f"未知(0x{control:02X})")
        result.command_word = command
        result.data_len = data_len
        result.data_hex = hex_with_spaces(data) if data else "无"
        result.checksum = checksum

        # 校验
        calc_sum = sum(frame[:6 + data_len]) & 0xFF
        result.checksum_valid = (calc_sum == checksum)

        # 描述
        parser = PARSER_MAP.get(control)
        if parser:
            result.description = parser(data, command)
        else:
            result.description = f"未知控制字 0x{control:02X}, 命令字 0x{command:02X}, 数据: {result.data_hex}"

        # 校验帧尾
        if tail != FRAME_TAIL:
            result.description += f" [警告: 帧尾异常, 期望 54 43, 实际 {hex_with_spaces(tail)}]"

        return result


# ======================== 日志记录器 ========================

class MdLogger:
    """Markdown 格式日志记录器，支持定时批量写入"""

    def __init__(self, log_dir: str = "./logs", batch_interval: float = 10.0):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.batch_interval = batch_interval
        self.queue: deque = deque()
        self.lock = threading.Lock()
        self._stop_event = threading.Event()
        self._current_file = None
        self._init_file()

        # 启动定时写入线程
        self._writer_thread = threading.Thread(target=self._batch_writer, daemon=True)
        self._writer_thread.start()

    def _init_file(self):
        """初始化当前日志文件"""
        today = datetime.now().strftime("%Y-%m-%d")
        self._current_file = self.log_dir / f"radar_log_{today}.md"
        if not self._current_file.exists():
            self._current_file.write_text(self._header(), encoding="utf-8")

    def _header(self) -> str:
        return (
            "# 云帆瑞达 60G 毫米波雷达数据日志\n\n"
            f"> 创建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n"
            "| 时间 | 控制字 | 控制字名称 | 命令字 | 数据长度 | 数据(Hex) | 校验 | 解析结果 |\n"
            "|------|--------|------------|--------|----------|-----------|------|----------|\n"
        )

    def add(self, frame: ParsedFrame):
        """添加一帧到写入队列"""
        with self.lock:
            self.queue.append(frame)

    def _batch_writer(self):
        """后台定时写入线程"""
        while not self._stop_event.is_set():
            self._stop_event.wait(self.batch_interval)
            self._flush()

    def _flush(self):
        """将队列中的数据写入文件"""
        with self.lock:
            if not self.queue:
                return

            # 检查是否跨天
            today = datetime.now().strftime("%Y-%m-%d")
            expected_file = self.log_dir / f"radar_log_{today}.md"
            if self._current_file != expected_file:
                self._current_file = expected_file
                if not self._current_file.exists():
                    self._current_file.write_text(self._header(), encoding="utf-8")

            lines = []
            while self.queue:
                frame = self.queue.popleft()
                d = frame.to_dict()
                ts = d["timestamp"]
                cw = d["control_word"]
                cn = d["control_name"]
                cmd = d["command_word"]
                dl = d["data_len"]
                dh = d["data_hex"]
                cv = d["checksum_valid"]
                desc = d["description"]
                # 转义 MD 表格中的 | 字符
                desc = desc.replace("|", "\\|")
                lines.append(f"| {ts} | {cw} | {cn} | {cmd} | {dl} | {dh} | {cv} | {desc} |\n")

            if lines:
                with open(self._current_file, "a", encoding="utf-8") as f:
                    f.writelines(lines)

    def flush_and_stop(self):
        """停止并写入剩余数据"""
        self._stop_event.set()
        self._writer_thread.join(timeout=5)
        self._flush()


# ======================== 终端打印 ========================

def print_frame(frame: ParsedFrame):
    """格式化打印解析结果到终端"""
    d = frame.to_dict()
    sep = "─" * 70
    print(f"\n{sep}")
    print(f"  时间:     {d['timestamp']}")
    print(f"  控制字:   {d['control_word']} ({d['control_name']})")
    print(f"  命令字:   {d['command_word']}")
    print(f"  数据长度: {d['data_len']} 字节")
    print(f"  数据Hex:  {d['data_hex']}")
    print(f"  校验:     {d['checksum']} ({d['checksum_valid']})")
    print(f"  解析:     {d['description']}")
    if not frame.checksum_valid:
        print(f"  ⚠ 校验失败!")
    print(sep)


# ======================== 主程序 ========================

def main():
    import argparse

    parser = argparse.ArgumentParser(description="云帆瑞达 60G 毫米波雷达串口数据解析")
    parser.add_argument("-p", "--port", default="COM3", help="串口端口 (默认: COM3)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="波特率 (默认: 115200)")
    parser.add_argument("-i", "--interval", type=float, default=10.0, help="日志批量写入间隔/秒 (默认: 10)")
    parser.add_argument("--log-dir", default="./logs", help="日志目录 (默认: ./logs)")
    parser.add_argument("--list-ports", action="store_true", help="列出可用串口")
    args = parser.parse_args()

    # 列出串口
    if args.list_ports:
        import serial.tools.list_ports as list_ports
        ports = list_ports.comports()
        if ports:
            print("可用串口:")
            for p in ports:
                print(f"  {p.device} - {p.description}")
        else:
            print("未检测到可用串口")
        return

    print("=" * 50)
    print("  云帆瑞达 60G 毫米波雷达串口解析程序")
    print("=" * 50)
    print(f"  串口: {args.port}")
    print(f"  波特率: {args.baud}")
    print(f"  日志目录: {args.log_dir}")
    print(f"  日志写入间隔: {args.interval}s")
    print("  按 Ctrl+C 停止")
    print("=" * 50)

    # 初始化
    frame_parser = FrameParser()
    logger = MdLogger(log_dir=args.log_dir, batch_interval=args.interval)

    # 打开串口
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )
        print(f"\n✓ 串口 {args.port} 已打开")
    except serial.SerialException as e:
        print(f"\n✗ 无法打开串口 {args.port}: {e}")
        print("  请检查:")
        print("  1. 串口是否被其他程序占用")
        print("  2. 是否使用正确的 COM 口 (使用 --list-ports 查看)")
        print("  3. 是否需要管理员权限")
        return

    print("\n开始接收数据...")

    try:
        while True:
            if ser.in_waiting > 0:
                raw = ser.read(ser.in_waiting)
                frames = frame_parser.feed(raw)
                for frame in frames:
                    print_frame(frame)
                    logger.add(frame)
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\n正在停止...")
    finally:
        logger.flush_and_stop()
        ser.close()
        print(f"✓ 日志已保存到: {logger._current_file}")
        print("✓ 串口已关闭")
        print("✓ 程序已退出")


if __name__ == "__main__":
    main()
