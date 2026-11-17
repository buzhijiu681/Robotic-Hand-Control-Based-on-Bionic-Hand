#ifndef __USART2_H
#define __USART2_H
#include "stm32f10x.h"

void USART2_Init(uint32_t baudrate);
void USART2_SendByte(uint8_t b);
void USART2_SendString(const char *s);
uint16_t USART2_RxAvailable(void);
uint8_t USART2_ReadByte(void);

#endif
