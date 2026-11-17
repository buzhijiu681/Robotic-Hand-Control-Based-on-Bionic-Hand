#include "delay.h"

/* 1ms tick 计数器，SysTick 中断每 1ms 自增 */
static volatile uint32_t g_tick_ms = 0;

/* 配置 SysTick 为 1ms 中断（72MHz / 1000 = 72000） */
void Delay_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000U);
}

void SysTick_Handler(void)
{
    g_tick_ms++;
}

uint32_t millis(void)
{
    return g_tick_ms;
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < ms) {
        /* 忙等，SysTick 中断会刷新 g_tick_ms */
    }
}

/* 粗略微秒级忙等（短延时用，不要求精确） */
void Delay_us(uint32_t us)
{
    volatile uint32_t i;
    while (us--) for (i = 0; i < 8; i++);
}
