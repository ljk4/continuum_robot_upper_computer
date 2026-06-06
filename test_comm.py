"""
test_comm.py -- 串口通信逐帧测试脚本

手动发送单帧，等待回复，排查通信链路。
不用协议库，直接构造原始字节，方便对照。

使用方法：
    conda activate continuum_robot
    python test_comm.py
"""

import struct
import time
import serial


def crc16_modbus(data: bytes) -> int:
    """CRC16-Modbus，与 protocol.py / protocol.c 完全一致"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(cmd: int, data: bytes = None) -> bytes:
    """构建一帧：[帧头 2B] [命令字 1B] [数据 32B] [CRC 2B] [帧尾 2B]"""
    if data is None:
        data = b'\x00' * 32
    if len(data) < 32:
        data = data + b'\x00' * (32 - len(data))

    frame = b'\xAA\x55' + bytes([cmd]) + data[:32]
    crc = crc16_modbus(frame)
    frame += struct.pack('<H', crc) + b'\x0D\x0A'
    return frame


def hex_dump(data: bytes) -> str:
    """字节序列转 HEX 字符串"""
    return ' '.join(f'{b:02X}' for b in data)


# =====================================================
# 配置（根据实际情况修改）
# =====================================================

PORT = "COM5"
BAUD = 115200
TIMEOUT = 2.0   # 等待回复超时 (秒)

# =====================================================
# 主测试流程
# =====================================================

def main():
    print("=" * 60)
    print("串口通信测试")
    print(f"端口: {PORT}  波特率: {BAUD}")
    print("=" * 60)

    # ---- 1. 打开串口 ----
    try:
        ser = serial.Serial(
            port=PORT, baudrate=BAUD, timeout=0.1,
            dsrdtr=False, rtscts=False
        )
        ser.setDTR(False)
        ser.setRTS(False)
        print(f"\n[OK] 串口 {PORT} 已打开")
    except Exception as e:
        print(f"\n[错误] 打开串口失败: {e}")
        return

    # ---- 2. 等待 STM32 就绪（监听反馈帧） ----
    print("\n等待 STM32 就绪...")
    ready = False
    t_wait = time.time()
    buf = bytearray()

    while time.time() - t_wait < 5.0:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buf.extend(data)
            # 搜索反馈帧头 0xAA55
            idx = buf.find(b'\xAA\x55')
            if idx >= 0 and len(buf) >= idx + 39:
                frame = bytes(buf[idx:idx + 39])
                if frame[2] == 0xA1:
                    print(f"[OK] 收到反馈帧，STM32 已就绪")
                    ready = True
                    break
        time.sleep(0.01)

    if not ready:
        print("[错误] 5 秒内未收到反馈帧，STM32 可能未运行")
        ser.close()
        return

    # ---- 3. 发送 CMD_QUERY 帧 ----
    print("\n" + "-" * 60)
    print("测试 1: 发送 CMD_QUERY (0x03)")
    print("-" * 60)

    frame = build_frame(0x03)
    print(f"发送 ({len(frame)} 字节): {hex_dump(frame)}")

    ser.write(frame)
    t_send = time.time()

    # 等待回复
    reply = bytearray()
    while time.time() - t_send < TIMEOUT:
        data = ser.read(ser.in_waiting or 1)
        if data:
            reply.extend(data)
            if len(reply) >= 39:
                break
        time.sleep(0.01)

    if reply:
        print(f"收到 ({len(reply)} 字节): {hex_dump(reply)}")

        if len(reply) >= 39 and reply[0] == 0xAA and reply[1] == 0x55:
            cmd = reply[2]
            recv_crc = reply[35] | (reply[36] << 8)
            calc_crc = crc16_modbus(bytes(reply[:35]))

            print(f"命令字: 0x{cmd:02X}  (期望 0xA1 反馈或 0x81 ACK)")
            print(f"CRC: 收到=0x{recv_crc:04X}  计算=0x{calc_crc:04X}  "
                  f"{'匹配' if recv_crc == calc_crc else '不匹配!'}")

            if cmd == 0xA1:
                values = struct.unpack('<8f', bytes(reply[3:35]))
                print(f"编码器值: {[f'{v:+.4f}' for v in values]}")
            elif cmd == 0x81:
                print("ACK 确认帧")
        else:
            print(f"[警告] 帧头不匹配，前 2 字节: {hex_dump(reply[:2])}")
    else:
        print("[超时] 未收到任何回复")

    # ---- 4. 发送 CMD_TARGET 帧（目标全零） ----
    print("\n" + "-" * 60)
    print("测试 2: 发送 CMD_TARGET (0x01)，目标全零")
    print("-" * 60)

    data = struct.pack('<8f', *[0.0] * 8)
    frame = build_frame(0x01, data)
    print(f"发送 ({len(frame)} 字节): {hex_dump(frame)}")

    ser.write(frame)
    t_send = time.time()

    reply = bytearray()
    while time.time() - t_send < TIMEOUT:
        d = ser.read(ser.in_waiting or 1)
        if d:
            reply.extend(d)
            if len(reply) >= 39:
                break
        time.sleep(0.01)

    if reply:
        print(f"收到 ({len(reply)} 字节): {hex_dump(reply)}")

        if len(reply) >= 39 and reply[0] == 0xAA and reply[1] == 0x55:
            cmd = reply[2]
            recv_crc = reply[35] | (reply[36] << 8)
            calc_crc = crc16_modbus(bytes(reply[:35]))

            print(f"命令字: 0x{cmd:02X}")
            print(f"CRC: 收到=0x{recv_crc:04X}  计算=0x{calc_crc:04X}  "
                  f"{'匹配' if recv_crc == calc_crc else '不匹配!'}")

            if cmd == 0x81:
                print("ACK 确认帧 - 通信正常!")
            elif cmd == 0xA1:
                values = struct.unpack('<8f', bytes(reply[3:35]))
                print(f"编码器值: {[f'{v:+.4f}' for v in values]}")
        else:
            print(f"[警告] 帧头不匹配，前 2 字节: {hex_dump(reply[:2])}")
    else:
        print("[超时] 未收到任何回复")

    # ---- 5. 持续监听 3 秒（诊断模式） ----
    print("\n" + "-" * 60)
    print("测试 3: 监听 3 秒（STM32 应以 20Hz 发送反馈帧）")
    print("-" * 60)

    # 诊断：先读 0.5 秒原始数据看看有没有字节进来
    print("诊断: 读取 0.5 秒原始数据...")
    diag_buf = bytearray()
    t_diag = time.time()
    while time.time() - t_diag < 0.5:
        n = ser.in_waiting
        if n > 0:
            diag_buf.extend(ser.read(n))
        time.sleep(0.001)

    print(f"诊断: 0.5 秒内收到 {len(diag_buf)} 字节")
    if len(diag_buf) > 0:
        print(f"诊断: 前 40 字节 HEX: {hex_dump(diag_buf[:40])}")
    else:
        print("诊断: 未收到任何数据! 串口可能已断开")

    # 正式监听
    t_start = time.time()
    fb_count = 0
    ack_count = 0
    raw_buf = bytearray()

    while time.time() - t_start < 3.0:
        n = ser.in_waiting
        if n > 0:
            raw_buf.extend(ser.read(n))

        while True:
            idx = raw_buf.find(b'\xAA\x55')
            if idx < 0:
                raw_buf.clear()
                break
            if len(raw_buf) < idx + 39:
                break

            f = bytes(raw_buf[idx:idx + 39])
            del raw_buf[:idx + 39]

            if len(f) == 39 and f[37] == 0x0D and f[38] == 0x0A:
                cmd = f[2]
                if cmd == 0xA1:
                    fb_count += 1
                elif cmd == 0x81:
                    ack_count += 1

        time.sleep(0.01)

    print(f"3 秒内收到: 反馈帧={fb_count}  ACK帧={ack_count}")

    if fb_count > 0:
        print("[OK] STM32 周期反馈正常")
    else:
        print("[异常] 未收到反馈帧")

    # ---- 6. 关闭 ----
    ser.close()
    print(f"\n[OK] 串口已关闭")


if __name__ == "__main__":
    main()
