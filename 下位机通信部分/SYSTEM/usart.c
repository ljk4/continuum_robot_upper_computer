#include "sys.h"
#include "usart.h"

/* =====================================================
 * printf 重定向（保留用于调试）
 * ===================================================== */
#if 1
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (u8)ch;
    return ch;
}
#endif

/* =====================================================
 * 环形缓冲区（ISR 写 head，主循环读 tail）
 * ===================================================== */
static volatile uint8_t  rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* =====================================================
 * 环形缓冲区访问函数
 * ===================================================== */
uint16_t uart_rx_count(void)
{
    return (rx_head - rx_tail) & (UART_RX_BUF_SIZE - 1);
}

uint8_t uart_rx_read(void)
{
    uint8_t data = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (UART_RX_BUF_SIZE - 1);
    return data;
}

void uart_rx_discard(uint16_t n)
{
    uint16_t cnt = uart_rx_count();
    if (n > cnt) n = cnt;
    rx_tail = (rx_tail + n) & (UART_RX_BUF_SIZE - 1);
}

/* =====================================================
 * 串口发送（阻塞轮询）
 * 115200 波特率下 39 字节约 3.4ms
 * ===================================================== */
void uart_send_frame(const uint8_t *frame, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART1->SR & 0x40) == 0);
        USART1->DR = frame[i];
    }
}

/* =====================================================
 * USART1 初始化
 * PA9(TX) / PA10(RX)，115200 8N1，RXNE 中断
 * ===================================================== */
void uart_init(u32 bound)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* =====================================================
 * USART1 中断服务函数
 * 仅做：读 DR → 存环形缓冲区 → 推进 head
 * ===================================================== */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);
        uint16_t next = (rx_head + 1) & (UART_RX_BUF_SIZE - 1);
        if (next != rx_tail) {
            rx_buf[rx_head] = data;
            rx_head = next;
        }
    }

    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET) {
        USART_ClearFlag(USART1, USART_FLAG_ORE);
    }
    USART_ClearITPendingBit(USART1, USART_IT_ORE);
}
