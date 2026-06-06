# utils/logger.py -- 日志模块：同时输出到控制台和文件

import os
import sys
import logging
from datetime import datetime


def setup_logger(name, log_dir="logs", level=logging.DEBUG):
    """创建一个同时输出到控制台和文件的 logger

    参数:
        name:    logger 名称（通常用模块名，如 "main"、"fake_stm32"）
        log_dir: 日志文件存放目录，相对于脚本所在目录
        level:   日志级别，默认 DEBUG

    文件输出:
        logs/{name}_{日期时间}.log
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # log_dir 相对于项目根目录（utils/ 的父目录）
    project_root = os.path.dirname(script_dir)
    full_log_dir = os.path.join(project_root, log_dir)
    os.makedirs(full_log_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(full_log_dir, f"{name}_{timestamp}.log")

    logger = logging.getLogger(name)
    logger.setLevel(level)

    if logger.handlers:
        return logger

    # 控制台 handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_fmt = logging.Formatter(
        "[%(asctime)s] %(levelname)s - %(message)s",
        datefmt="%H:%M:%S"
    )
    console_handler.setFormatter(console_fmt)

    # 文件 handler
    file_handler = logging.FileHandler(log_file, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_fmt = logging.Formatter(
        "[%(asctime)s.%(msecs)03d] [%(levelname)-5s] "
        "[%(filename)s:%(lineno)d] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    file_handler.setFormatter(file_fmt)

    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    return logger
