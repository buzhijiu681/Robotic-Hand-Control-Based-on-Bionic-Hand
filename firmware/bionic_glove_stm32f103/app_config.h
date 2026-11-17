#ifndef __DEMO_APP_CONFIG_H
#define __DEMO_APP_CONFIG_H

/* ===================== 串口 ===================== */
#define DEMO_USB_BAUD           115200
#define DEMO_WIFI_BAUD          115200

/* ===================== ESP8266服务器 ===================== */
#define DEMO_WIFI_AP_SSID       "PYY_8266"
#define DEMO_WIFI_AP_PASSWORD   "12345678"
#define DEMO_WIFI_TCP_PORT      80

#define DEMO_STRINGIFY_INNER(value)  #value
#define DEMO_STRINGIFY(value)        DEMO_STRINGIFY_INNER(value)
#define DEMO_WIFI_TCP_PORT_TEXT      DEMO_STRINGIFY(DEMO_WIFI_TCP_PORT)
#define DEMO_VERSION                 "2026-10-29-r22"

/* ===================== 手套数据 ===================== */
#define DEMO_SENSOR_COUNT       5
#define DEMO_SENSOR_LINE_SIZE   128
#define DEMO_WIFI_LINE_SIZE     192
#define DEMO_SENSOR_SAMPLE_MS   10
#define DEMO_WIFI_SEND_MS       100
#define DEMO_DISPLAY_MS         250
#define DEMO_WIFI_RETRY_MS      5000
#define DEMO_WIFI_RECOVER_MS    1000
#define DEMO_ALL_BENT_ON        70
#define DEMO_ALL_BENT_OFF       45
#define DEMO_S3_TRIGGER_ON      95
#define DEMO_S3_TRIGGER_OFF     90
#define DEMO_BEND_DEADBAND      5
#define DEMO_BEND_CONFIRM       2
#define DEMO_ADC_VALID_MIN      5
#define DEMO_ADC_VALID_MAX      4090

/* S2弯曲百分比单独放大，保持张开值0%，避免固定偏移造成误动作 */
#define DEMO_S2_GAIN_PERCENT    135
#define DEMO_S2_OPEN_DEADBAND   80
#define DEMO_S2_FULL_SCALE_ADC  160

#endif
