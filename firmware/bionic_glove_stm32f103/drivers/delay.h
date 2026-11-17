#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f10x.h"

void Delay_Init(void);        /* 开 SysTick 1ms 中断 */
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);
uint32_t millis(void);        /* 上电以来的毫秒数 */

#endif
