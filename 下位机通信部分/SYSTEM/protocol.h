#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "sys.h"

/* ---- 帧布局常量 ---- */
#define FRAME_HEADER_0      0xAA
#define FRAME_HEADER_1      0x55
#define FRAME_TAIL_0        0x0D
#define FRAME_TAIL_1        0x0A
#define FRAME_LEN           39      /* 2+1+32+2+2 */
#define FRAME_DATA_LEN      32      /* 8 x float32 */
#define FRAME_CRC_OFFSET    35
#define FRAME_DATA_OFFSET   3

/* ---- 命令字 ---- */
#define CMD_TARGET          0x01    /* PC->STM32: 目标值 */
#define CMD_STOP            0x02    /* PC->STM32: 急停 */
#define CMD_QUERY           0x03    /* PC->STM32: 查询状态 */
#define CMD_ACK             0x81    /* STM32->PC: 应答确认 */
#define CMD_FEEDBACK        0xA1    /* STM32->PC: 编码器反馈 */

#define NUM_CABLES          8

/* ---- CRC16-Modbus ---- */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

/* ---- 帧打包 (STM32 -> PC) ---- */
void protocol_pack_ack(uint8_t *buf);
void protocol_pack_feedback(uint8_t *buf, const float *values);

/* ---- 帧校验与解析 (PC -> STM32) ---- */
uint8_t protocol_verify(const uint8_t *frame);
uint8_t protocol_parse_cmd(const uint8_t *frame);
void    protocol_parse_data(const uint8_t *frame, float *out);

#endif
