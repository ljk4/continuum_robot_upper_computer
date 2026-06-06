# comm/sender.py -- 100Hz 发送线程

import time
import threading

from comm.protocol import pack_target
from config import control_cfg as cfg


class SenderThread(threading.Thread):
    def __init__(self, serial_mgr):
        super().__init__(daemon=True)
        self.serial_mgr = serial_mgr
        self.running = True
        self.target = [0.0] * 8
        self.lock = threading.Lock()

    def update_target(self, lengths):
        with self.lock:
            self.target = lengths.copy()

    def stop(self):
        self.running = False

    def run(self):
        freq = cfg.send_hz
        period = 1.0 / freq

        while self.running:
            try:
                t0 = time.perf_counter()

                with self.lock:
                    target = self.target.copy()

                frame = pack_target(target)
                self.serial_mgr.send(frame)

                dt = time.perf_counter() - t0
                remain = period - dt
                if remain > 0:
                    time.sleep(remain)

            except Exception:
                break  # 串口关闭后抛异常，线程正常退出
