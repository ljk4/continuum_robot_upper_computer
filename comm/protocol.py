# comm/protocol.py -- 串口通信协议：帧打包/解包/CRC16-Modbus

import struct

from config import protocol_cfg as cfg

FRAME_HEADER = cfg.frame_header
FRAME_TAIL = cfg.frame_tail

CMD_TARGET = cfg.cmd_target
CMD_STOP = cfg.cmd_stop
CMD_QUERY = cfg.cmd_query

CMD_FEEDBACK = cfg.cmd_feedback
CMD_ACK = cfg.cmd_ack

FRAME_LEN = cfg.frame_len


def crc16_modbus(data: bytes):
    """CRC16-Modbus，多项式 0xA001"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ cfg.crc_polynomial
            else:
                crc >>= 1
    return crc & 0xFFFF


def pack_target(lengths):
    """打包目标值帧：8 个 float32（舵机圈数）"""
    if len(lengths) != 8:
        raise ValueError("lengths must contain 8 floats")

    frame = bytearray()
    frame += FRAME_HEADER
    frame.append(CMD_TARGET)
    frame += struct.pack("<8f", *lengths)
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    frame += FRAME_TAIL
    return bytes(frame)


def pack_stop():
    """打包急停帧"""
    frame = bytearray()
    frame += FRAME_HEADER
    frame.append(CMD_STOP)
    frame += bytes(32)
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    frame += FRAME_TAIL
    return bytes(frame)


def pack_query():
    """打包状态查询帧"""
    frame = bytearray()
    frame += FRAME_HEADER
    frame.append(CMD_QUERY)
    frame += bytes(32)
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    frame += FRAME_TAIL
    return bytes(frame)


def verify_frame(frame):
    """校验帧完整性（帧头、帧尾、CRC）"""
    if len(frame) != FRAME_LEN:
        return False
    if frame[0:2] != FRAME_HEADER:
        return False
    if frame[-2:] != FRAME_TAIL:
        return False

    recv_crc = struct.unpack("<H", frame[35:37])[0]
    calc_crc = crc16_modbus(frame[:35])
    return recv_crc == calc_crc


def parse_frame(frame):
    """解析帧，返回 (cmd, values) 或 None"""
    if not verify_frame(frame):
        return None

    cmd = frame[2]
    values = struct.unpack("<8f", frame[3:35])
    return cmd, values
