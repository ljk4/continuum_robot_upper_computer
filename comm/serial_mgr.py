# comm/serial_mgr.py -- 线程安全串口管理

import time
import serial
import threading

from config import serial_cfg as cfg
from utils.logger import setup_logger

log = setup_logger("serial_mgr")


class SerialManager:
    def __init__(self, port=None, baudrate=None, wait_ready=True):
        if port is None:
            port = cfg.port_main
        if baudrate is None:
            baudrate = cfg.baudrate

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=cfg.timeout,
            write_timeout=cfg.write_timeout,
            dsrdtr=False,
            rtscts=False
        )
        try:
            self.ser.setDTR(False)
            self.ser.setRTS(False)
        except Exception:
            pass  # 部分 USB 串口适配器不支持硬件流控

        self.tx_lock = threading.Lock()

        if wait_ready:
            self._wait_stm32_ready()

    def _wait_stm32_ready(self, timeout=5.0):
        log.info("等待 STM32 就绪...")
        buf = bytearray()
        t0 = time.time()

        while time.time() - t0 < timeout:
            n = self.ser.in_waiting
            if n > 0:
                buf.extend(self.ser.read(n))
                idx = buf.find(b'\xAA\x55')
                if idx >= 0 and len(buf) >= idx + 39:
                    if buf[idx + 2] == 0xA1:
                        log.info("STM32 已就绪")
                        return
            time.sleep(0.01)

        log.warning("等待 STM32 超时，继续运行...")

    def send(self, data: bytes):
        with self.tx_lock:
            self.ser.write(data)

    def read_available(self):
        n = self.ser.in_waiting
        if n > 0:
            return self.ser.read(n)
        return b''

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass
