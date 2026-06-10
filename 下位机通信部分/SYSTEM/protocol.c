#include "protocol.h"
#include <string.h>

/* =====================================================
 * CRC16-Modbus
 * 多项式 0xA001，初值 0xFFFF
 * 移植自 protocol.py:20-39
 * ===================================================== */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* =====================================================
 * 帧打包辅助：填充帧头、CRC、帧尾
 * buf 至少 FRAME_LEN 字节
 * 调用前已写好 cmd (buf[2]) 和 data (buf[3..34])
 * ===================================================== */
static void frame_finalize(uint8_t *buf)
{
    uint16_t crc;

    buf[0] = FRAME_HEADER_0;
    buf[1] = FRAME_HEADER_1;

    crc = crc16_modbus(buf, 35);
    buf[35] = (uint8_t)(crc & 0xFF);
    buf[36] = (uint8_t)(crc >> 8);

    buf[37] = FRAME_TAIL_0;
    buf[38] = FRAME_TAIL_1;
}

/* =====================================================
 * 构建 ACK 应答帧
 * ===================================================== */
void protocol_pack_ack(uint8_t *buf)
{
    memset(buf, 0, FRAME_LEN);
    buf[2] = CMD_ACK;
    frame_finalize(buf);
}

/* =====================================================
 * 构建编码器反馈帧
 * values: 8 个 float，按 绳1~绳8 顺序
 * Cortex-M4 小端序，直接 memcpy
 * ===================================================== */
void protocol_pack_feedback(uint8_t *buf, const float *values)
{
    memset(buf, 0, FRAME_LEN);
    buf[2] = CMD_FEEDBACK;
    memcpy(&buf[3], values, FRAME_DATA_LEN);
    frame_finalize(buf);
}

/* =====================================================
 * 帧校验：检查帧头、帧尾、CRC
 * 返回 1 有效，0 无效
 * ===================================================== */
uint8_t protocol_verify(const uint8_t *frame)
{
    uint16_t recv_crc;
    uint16_t calc_crc;

    if (frame[0] != FRAME_HEADER_0 || frame[1] != FRAME_HEADER_1)
        return 0;

    if (frame[37] != FRAME_TAIL_0 || frame[38] != FRAME_TAIL_1)
        return 0;

    recv_crc = (uint16_t)frame[35] | ((uint16_t)frame[36] << 8);
    calc_crc = crc16_modbus(frame, 35);

    return (recv_crc == calc_crc) ? 1 : 0;
}

/* =====================================================
 * 解析命令字节
 * ===================================================== */
uint8_t protocol_parse_cmd(const uint8_t *frame)
{
    return frame[2];
}

/* =====================================================
 * 解析数据区为 8 个 float
 * ===================================================== */
void protocol_parse_data(const uint8_t *frame, float *out)
{
    memcpy(out, &frame[3], FRAME_DATA_LEN);
}
