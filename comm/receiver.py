# comm/receiver.py -- 接收线程：帧解析 + ACK/反馈分发

import time
import threading

from comm.protocol import (
    FRAME_LEN,
    FRAME_HEADER,
    parse_frame,
    CMD_FEEDBACK,
    CMD_ACK,
)
from config import robot_cfg
from utils.logger import setup_logger

log = setup_logger("receiver")


class ReceiverThread(threading.Thread):
    def __init__(self, serial_mgr):
        super().__init__(daemon=True)
        self.serial_mgr = serial_mgr
        self.running = True
        self.buffer = bytearray()
        self.latest_encoder = None
        self.last_ack_time = time.time()

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            try:
                data = self.serial_mgr.read_available()
            except Exception:
                break  # 串口关闭后抛异常，线程正常退出

            if not data:
                continue

            self.buffer.extend(data)
            log.debug("收到 %d 字节，缓冲区 %d 字节",
                      len(data), len(self.buffer))

            while True:
                idx = self.buffer.find(FRAME_HEADER)

                if idx < 0:
                    # 不清空全部，保留最后 (帧头长度-1) 字节，
                    # 防止帧头跨 read 边界时丢失数据
                    keep = len(FRAME_HEADER) - 1
                    if len(self.buffer) > keep:
                        del self.buffer[:len(self.buffer) - keep]
                    break

                # 丢弃帧头之前的无效字节
                if idx > 0:
                    del self.buffer[:idx]

                if len(self.buffer) < FRAME_LEN:
                    break

                frame = self.buffer[:FRAME_LEN]
                del self.buffer[:FRAME_LEN]

                result = parse_frame(frame)
                if result is None:
                    continue

                cmd, values = result

                if cmd == CMD_FEEDBACK:
                    self.latest_encoder = values[:robot_cfg.num_cables]
                    log.debug("收到反馈帧")

                elif cmd == CMD_ACK:
                    self.last_ack_time = time.time()
                    log.debug("收到 ACK")
