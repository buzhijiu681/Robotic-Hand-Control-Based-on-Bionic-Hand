#ifndef __COMM_H
#define __COMM_H

#include "stm32f4xx.h"

/* 支持的控制来源：电脑串口、蓝牙、WiFi */
typedef enum
{
    COMM_SOURCE_SERIAL = 0,
    COMM_SOURCE_BLUETOOTH,
    COMM_SOURCE_WIFI,
    COMM_SOURCE_COUNT
} CommSource;

void Comm_Init(void);
void Comm_InitBluetooth(uint32_t baud);
uint32_t Comm_GetBluetoothBaud(void);
void Comm_RxByteFromISR(CommSource source, uint8_t byte);
uint8_t Comm_ReadLine(CommSource source, char *line, uint16_t line_size);
uint32_t Comm_GetRxByteCount(CommSource source);
void Comm_Send(CommSource source, const char *text);
void Comm_SendLine(CommSource source, const char *text);
void Comm_Clear(CommSource source);
const char *Comm_SourceName(CommSource source);

#endif
