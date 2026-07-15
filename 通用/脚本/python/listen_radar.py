"""
深入分析数据流中 53 59 的上下文
检查是否可能需要不同的解析方式
"""
import sys, codecs, serial, time, struct
if sys.platform == 'win32':
    try: sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    except: pass

ser = serial.Serial('COM3', 115200, timeout=1)
print('串口已打开, 收集数据5秒...')

ser.reset_input_buffer()
all_data = bytearray()
start = time.time()
while time.time() - start < 5:
    if ser.in_waiting > 0:
        raw = ser.read(ser.in_waiting)
        all_data.extend(raw)
    time.sleep(0.01)

print('收到 {} 字节\n'.format(len(all_data)))

# 查找所有 53 59 位置，并显示上下文
print('=== 53 59 帧头分析 ===')
positions = []
for i in range(len(all_data) - 1):
    if all_data[i] == 0x53 and all_data[i+1] == 0x59:
        positions.append(i)

print('共找到 {} 处\n'.format(len(positions)))

for idx, pos in enumerate(positions):
    context_start = max(0, pos - 5)
    context_end = min(len(all_data), pos + 20)
    context = all_data[context_start:context_end]

    print('--- 位置 {} (idx={}) ---'.format(pos, idx))
    # 显示上下文，标记帧头位置
    line = ''
    for j, b in enumerate(context):
        abs_pos = context_start + j
        if abs_pos == pos:
            line += '[53 59] '
            continue
        elif abs_pos == pos + 1:
            continue
        line += '{:02X} '.format(b)
    print('  上下文: {}'.format(line))

    if pos + 7 <= len(all_data):
        ctrl = all_data[pos+2]
        cmd = all_data[pos+3]
        dlen = struct.unpack('>H', all_data[pos+4:pos+6])[0]
        print('  ctrl=0x{:02X} cmd=0x{:02X} dlen={}'.format(ctrl, cmd, dlen))

# 也查找 54 43 帧尾
print('\n=== 54 43 帧尾分析 ===')
tail_positions = []
for i in range(len(all_data) - 1):
    if all_data[i] == 0x54 and all_data[i+1] == 0x43:
        tail_positions.append(i)
print('共找到 {} 处 54 43'.format(len(tail_positions)))

# 看看是否有固定模式
# 比如 5B 5D 开头的数据包
print('\n=== 5B 5D 数据包分析 ===')
count_5b5d = 0
for i in range(len(all_data) - 1):
    if all_data[i] == 0x5B and all_data[i+1] == 0x5D:
        count_5b5d += 1
print('5B 5D 出现次数: {}'.format(count_5b5d))

# 检查 5B ... 7B 模式 (类似TLV结构)
print('\n=== 数据包边界分析 ===')
# 检查是否有固定长度的数据包
if len(all_data) > 20:
    # 检查 57 7B 是否是某种结束标记
    count_577b = 0
    for i in range(len(all_data) - 1):
        if all_data[i] == 0x57 and all_data[i+1] == 0x7B:
            count_577b += 1
    print('57 7B 出现次数: {}'.format(count_577b))

    # 看看数据是否以固定模式分组
    # 先看前200字节的结构
    print('\n前200字节(每行20字节):')
    for i in range(0, min(200, len(all_data)), 20):
        chunk = all_data[i:i+20]
        print('  {:4d}: {}'.format(i, ' '.join('{:02X}'.format(b) for b in chunk)))

ser.close()
print('\n串口已关闭')
