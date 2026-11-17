#include "usart2.h"

static uint8_t rxbuf[512];
static volatile uint16_t rx_w = 0, rx_r = 0;

/* USART2: PA2=TX, PA3=RX  —— 接 ESP8266 */
void USART2_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate            = baudrate;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void USART2_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, b);
}

void USART2_SendString(const char *s)
{
    while (*s) USART2_SendByte((uint8_t)*s++);
}

uint16_t USART2_RxAvailable(void)
{
    return (rx_w - rx_r) & 0x1FF;
}

uint8_t USART2_ReadByte(void)
{
    uint8_t b = rxbuf[rx_r];
    rx_r = (rx_r + 1) & 0x1FF;
    return b;
}

/* USART2 收到字节 → 存环形缓冲（不转发到 USART1，避免中断死循环） */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint16_t data = USART_ReceiveData(USART2);
        uint16_t next = (rx_w + 1) & 0x1FF;
        if (next != rx_r) {
            rxbuf[rx_w] = (uint8_t)data;
            rx_w = next;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
