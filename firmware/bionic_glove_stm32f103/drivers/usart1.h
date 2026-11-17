#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"

void USART1_Init(uint32_t baudrate);
void USART1_SendByte(uint8_t b);
void USART1_SendString(const char *s);
uint16_t USART1_RxAvailable(void);
uint8_t USART1_ReadByte(void);

#endif
