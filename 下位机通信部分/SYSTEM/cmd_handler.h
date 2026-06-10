#ifndef __CMD_HANDLER_H
#define __CMD_HANDLER_H

#include "sys.h"
#include "protocol.h"

/* ---- 共享状态 ---- */
extern float   target_rotations[NUM_CABLES];
extern float   encoder_rotations[NUM_CABLES];
extern uint8_t system_stopped;

/* ---- 初始化 ---- */
void cmd_handler_init(void);

/* ---- 轮询处理 UART 接收帧（每轮主循环调用） ---- */
void cmd_handler_poll(void);

/* ---- 周期反馈（20Hz 调用） ---- */
void cmd_handler_send_feedback(void);

#endif
