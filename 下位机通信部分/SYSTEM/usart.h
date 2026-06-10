#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

/* ---- 环形缓冲区 ---- */
#define UART_RX_BUF_SIZE    512     /* 2 的幂次，ISR 写、主循环读 */

/* ---- API ---- */
void     uart_init(u32 bound);
void     uart_send_frame(const uint8_t *frame, uint16_t len);

uint16_t uart_rx_count(void);
uint8_t  uart_rx_read(void);
void     uart_rx_discard(uint16_t n);

#endif
