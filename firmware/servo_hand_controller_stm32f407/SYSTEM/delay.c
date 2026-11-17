#include "delay.h"

/*
 * 原工程按21 MHz写成固定装载值，当前STM32F407运行在168 MHz，
 * 会导致所有毫秒和微秒延时缩短约8倍。这里按实际主频计算装载值。
 */
static uint32_t Delay_GetReload(uint32_t cycles)
{
	uint32_t reload;

    if(cycles == 0)
        cycles = 1;
    reload = cycles - 1U;
    return reload;
}

void delay_ms(uint16_t ms)
{
	uint32_t reload = Delay_GetReload(SystemCoreClock / 1000U);

	while(ms--)
	{
		SysTick->CTRL = 0; 			// 禁用系统滴答时钟
		SysTick->LOAD = reload; 	// 按实际主频生成1 ms装载值
		SysTick->VAL = 0; 			// 清除当前值并清空计数标志
		SysTick->CTRL = 1; 			// 使能系统滴答时钟开始工作
		//bit0给1表示使能系统滴答时钟
		while ((SysTick->CTRL & 0x00010000) == 0);// 等待中断标志位为真
	}
	SysTick->CTRL = 0; 			// 禁用系统滴答时钟
}

void delay_us(uint16_t us)
{
	uint32_t reload = Delay_GetReload(SystemCoreClock);

	while(us--)
	{
		SysTick->CTRL = 0; 			// 禁用系统滴答时钟
		SysTick->LOAD = reload; 	// 按实际主频生成1 us装载值
		SysTick->VAL = 0; 			// 清除当前值并清空计数标志
		SysTick->CTRL = 1; 			// 使能系统滴答时钟开始工作
		//bit0给1表示使能系统滴答时钟
		while ((SysTick->CTRL & 0x00010000) == 0);// 等待中断标志位为真
	}
	SysTick->CTRL = 0; 			// 禁用系统滴答时钟
}

void delay_s(uint16_t s)
{
	while(s--)
	{
		delay_ms(1000);
	}
}




