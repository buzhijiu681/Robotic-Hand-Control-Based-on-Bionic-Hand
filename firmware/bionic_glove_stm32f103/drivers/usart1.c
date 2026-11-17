#include "usart1.h"

static uint8_t rxbuf[512];
static volatile uint16_t rx_w = 0;
static volatile uint16_t rx_r = 0;

/* USART1：PA9 TX / PA10 RX，连接 USB-TTL */
void USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,
                           ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_SendByte(uint8_t b)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, b);
}

void USART1_SendString(const char *s)
{
    while(*s != '\0')
        USART1_SendByte((uint8_t)*s++);
}

uint16_t USART1_RxAvailable(void)
{
    return (rx_w - rx_r) & 0x1FF;
}

uint8_t USART1_ReadByte(void)
{
    uint8_t b = rxbuf[rx_r];
    rx_r = (rx_r + 1) & 0x1FF;
    return b;
}

/* 接收数据只写入环形缓冲区，由主循环解析，避免中断内阻塞 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint16_t data = USART_ReceiveData(USART1);
        uint16_t next = (rx_w + 1) & 0x1FF;

        if(next != rx_r)
        {
            rxbuf[rx_w] = (uint8_t)data;
            rx_w = next;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
        (void)USART_ReceiveData(USART1);
}
