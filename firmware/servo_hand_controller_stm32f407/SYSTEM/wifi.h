#ifndef __WIFI_H
#define __WIFI_H

#include "stm32f4xx.h"

/* 舵机端ESP8266作为STA客户端，连接demo的AP和TCP服务器 */
#define WIFI_STA_SSID          "PYY_8266"
#define WIFI_STA_PASSWORD      "12345678"
#define WIFI_SERVER_IP         "192.168.4.1"
#define WIFI_TCP_PORT          80
#define WIFI_UART_BAUD         115200

#define WIFI_STRINGIFY_INNER(value)  #value
#define WIFI_STRINGIFY(value)        WIFI_STRINGIFY_INNER(value)
#define WIFI_TCP_PORT_TEXT           WIFI_STRINGIFY(WIFI_TCP_PORT)

typedef enum
{
    WIFI_STATE_NO_MODULE = 0,
    WIFI_STATE_AP_NOT_JOINED,
    WIFI_STATE_WAITING_TCP,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_DISABLED
} WiFiState;

uint8_t WiFi_Init(void);
uint8_t WiFi_SetBaud(uint32_t baud);
uint32_t WiFi_GetBaud(void);
void WiFi_Suspend(void);
void WiFi_Resume(void);
uint8_t WiFi_ATCommand(const char *command,
                       char *response,
                       uint16_t response_size,
                       uint16_t timeout_ms);
void WiFi_RxByteFromISR(uint8_t byte);
void WiFi_Poll(uint16_t elapsed_ms);
void WiFi_NotifyReceived(void);
uint8_t WiFi_HandleNotice(const char *line);
uint8_t WiFi_IsConnected(void);
uint8_t WiFi_IsServerReady(void);
WiFiState WiFi_GetState(void);

#endif
