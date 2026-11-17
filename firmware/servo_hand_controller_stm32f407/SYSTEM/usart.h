#ifndef _UASRT_H__
#define _UASRT_H__

#include "stm32f4xx.h"
#include "stdio.h"

//USART1	TX1	PA9
//USART1	RX1	PA10
void init_usart1(uint32_t baud);
void send_char(uint16_t ch);
void send_str(char *str);
//编写 中断服务函数（用来接收数据）
void USART1_IRQHandler(void);


void init_usart2(uint32_t baud);
void send2_char(uint16_t ch);
void send2_str(char *str);
void USART2_IRQHandler(void);


void init_usart3(uint32_t baud);
void send3_char(uint16_t ch);
void send3_str(char *str);
void USART3_IRQHandler(void);


void init_usart6(uint32_t baud);
void send6_char(uint16_t ch);
void send6_str(char *str);
void USART6_IRQHandler(void);




#endif


