#ifndef __PCA9685_H
#define __PCA9685_H
#include "sys.h"   // 按你工程实际的头文件改，如 myhead.h

/* MG996R 常见标准脉宽范围；个别舵机可在500~2500us内微调 */
#define PCA9685_SERVO_MIN_US  500
#define PCA9685_SERVO_MAX_US  2500

void PCA9685_Init(void);
void PCA9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off);
void PCA9685_SetPulseUs(uint8_t ch, uint16_t pulse_us);
void PCA9685_SetChannelOff(uint8_t ch);
void PCA9685_SetAngle(uint8_t ch, uint8_t angle);  // ch:0~15, angle:0~180

#endif

