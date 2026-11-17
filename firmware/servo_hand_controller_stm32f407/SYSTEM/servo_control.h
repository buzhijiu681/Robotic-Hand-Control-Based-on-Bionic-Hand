#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#include "stm32f4xx.h"
#include "app_config.h"

typedef struct
{
    uint8_t enabled;
    uint8_t current_angle;
    uint8_t target_angle;
    uint8_t min_angle;
    uint8_t max_angle;
} ServoPositionStatus;

typedef struct
{
    uint8_t speed_cmd;
    uint16_t pulse_us;
    uint16_t stop_us;
    uint8_t raw_pulse_mode;
    uint8_t output_disabled;
} ServoContinuousStatus;

void Servo_ControlInit(void);
uint8_t Servo_SetChannelValue(uint8_t ch, int value);
uint8_t Servo_SetAllValue(int value);
void Servo_UpdatePositions(void);
void Servo_UpdateDemo(void);
void Servo_StartDemo(void);
void Servo_StartDemoOnce(void);
void Servo_StartContinuousDemo(void);
void Servo_StartContinuousOnce(void);
void Servo_StopAll(void);
void Servo_GoHome(void);
void Servo_DisableContinuousOutput(void);
uint8_t Servo_SetSensorBends(
    const uint8_t bends[APP_SENSOR_COUNT]);
void Servo_GetSensorBends(uint8_t bends[APP_SENSOR_COUNT],
                          uint8_t *valid);
uint8_t Servo_IsDemoRunning(void);

uint8_t Servo_SetContinuousTrim(int trim_us);
uint8_t Servo_GetPositionStatus(uint8_t ch, ServoPositionStatus *status);
void Servo_GetContinuousStatus(ServoContinuousStatus *status);

#endif
