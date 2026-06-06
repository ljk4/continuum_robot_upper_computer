# receiver.py 负责：接收反馈 解包 保存编码器值

import threading

from protocol import (
    FRAME_LEN,
    parse_frame,
    CMD_FEEDBACK,
    CMD_ACK
)
from logger import setup_logger

log = setup_logger("receiver")


class ReceiverThread(threading.Thread):

    def __init__(self, serial_mgr):

        super().__init__(daemon=True)

        self.serial_mgr = serial_mgr

        self.running = True

        self.buffer = bytearray()

        self.latest_encoder = None

        self.last_ack_time = __import__("time").time()

    def stop(self):

        self.running = False

    def run(self):

        while self.running:

            try:
                data = self.serial_mgr.read_available()
            except Exception:
                break  # 串口关闭后退出

            if not data:
                continue

            self.buffer.extend(data)
            log.debug("收到 %d 字节，缓冲区 %d 字节",
                      len(data), len(self.buffer))

            while True:

                idx = self.buffer.find(
                    b'\xAA\x55'
                )

                if idx < 0:

                    self.buffer.clear()
                    break

                if len(self.buffer) < idx + FRAME_LEN:
                    break

                frame = self.buffer[
                    idx:idx + FRAME_LEN
                ]

                del self.buffer[
                    :idx + FRAME_LEN
                ]

                result = parse_frame(frame)

                if result is None:
                    continue

                cmd, values = result

                if cmd == CMD_FEEDBACK:

                    self.latest_encoder = values
                    log.debug("收到反馈帧")

                elif cmd == CMD_ACK:

                    self.last_ack_time = __import__(
                        "time"
                    ).time()
                    log.debug("收到 ACK")