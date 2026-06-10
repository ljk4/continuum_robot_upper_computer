#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "protocol.h"

/* ---- 初始化 ---- */
void control_init(void);

/* ---- PID 伺服更新（每次主循环调用） ---- */
void control_update(void);

/* ---- 急停/使能 ---- */
void control_set_enable(uint8_t en);

#endif
