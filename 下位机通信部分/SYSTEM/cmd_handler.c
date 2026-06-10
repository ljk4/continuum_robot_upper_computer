#include "cmd_handler.h"
#include "control.h"
#include "usart.h"
#include <string.h>

/* =====================================================
 * 共享状态
 * ===================================================== */
float   target_rotations[NUM_CABLES]  = {0.0f};
float   encoder_rotations[NUM_CABLES] = {0.0f};
uint8_t system_stopped = 0;

/* =====================================================
 * 帧组装缓冲区
 * ===================================================== */
#define ASM_BUF_SIZE    78  /* 2 帧 */

static uint8_t  asm_buf[ASM_BUF_SIZE];
static uint16_t asm_len = 0;

/* =====================================================
 * 初始化
 * ===================================================== */
void cmd_handler_init(void)
{
    uint8_t i;
    for (i = 0; i < NUM_CABLES; i++) {
        target_rotations[i]  = 0.0f;
        encoder_rotations[i] = 0.0f;
    }
    system_stopped = 0;
    asm_len = 0;
}

/* =====================================================
 * 命令分发（内部函数）
 * ===================================================== */
static void dispatch_frame(const uint8_t *frame)
{
    uint8_t cmd = protocol_parse_cmd(frame);
    uint8_t ack_frame[FRAME_LEN];
    float values[NUM_CABLES];
    uint8_t i;

    switch (cmd) {

    case CMD_TARGET:
        protocol_parse_data(frame, values);
        for (i = 0; i < NUM_CABLES; i++) {
            target_rotations[i] = values[i];
        }
        control_set_enable(1);  /* 收到新目标，恢复使能 */
        protocol_pack_ack(ack_frame);
        uart_send_frame(ack_frame, FRAME_LEN);
        break;

    case CMD_STOP:
        control_set_enable(0);  /* 急停：锁定位置，电机失能 */
        //oled_display_update();
        protocol_pack_ack(ack_frame);
        uart_send_frame(ack_frame, FRAME_LEN);
        break;

    case CMD_QUERY:
        protocol_pack_feedback(ack_frame, encoder_rotations);
        uart_send_frame(ack_frame, FRAME_LEN);
        break;

    default:
        break;
    }
}

/* =====================================================
 * 在组装缓冲区中搜索帧头 0xAA 0x55
 * 返回索引，未找到返回 -1
 * ===================================================== */
static int16_t find_header(void)
{
    uint16_t i;
    for (i = 0; i + 1 < asm_len; i++) {
        if (asm_buf[i] == FRAME_HEADER_0 && asm_buf[i + 1] == FRAME_HEADER_1)
            return (int16_t)i;
    }
    return -1;
}

/* =====================================================
 * 从组装缓冲区前端删除 n 字节
 * ===================================================== */
static void asm_discard(uint16_t n)
{
    if (n >= asm_len) {
        asm_len = 0;
        return;
    }
    memmove(asm_buf, asm_buf + n, asm_len - n);
    asm_len -= n;
}

/* =====================================================
 * 轮询处理：读取环形缓冲区 → 组装帧 → 校验 → 分发
 * ===================================================== */
void cmd_handler_poll(void)
{
    int16_t idx;
    uint8_t frame[FRAME_LEN];

    while (uart_rx_count() > 0 && asm_len < ASM_BUF_SIZE) {
        asm_buf[asm_len++] = uart_rx_read();
    }

    while (1) {
        idx = find_header();
        if (idx < 0) {
            asm_len = 0;
            break;
        }

        if (idx > 0) {
            asm_discard((uint16_t)idx);
        }

        if (asm_len < FRAME_LEN)
            break;

        memcpy(frame, asm_buf, FRAME_LEN);
        asm_discard(FRAME_LEN);

        if (!protocol_verify(frame))
            continue;

        dispatch_frame(frame);
    }
}

/* =====================================================
 * 周期发送编码器反馈（20Hz）
 * ===================================================== */
void cmd_handler_send_feedback(void)
{
    uint8_t frame[FRAME_LEN];
    protocol_pack_feedback(frame, encoder_rotations);
    uart_send_frame(frame, FRAME_LEN);
}
