#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ===================== 通信与主循环 ===================== */
#define APP_SERIAL_BAUD             115200
#define APP_BLUETOOTH_BAUD          9600
#define APP_TICK_MS                 10
#define APP_COMMAND_SIZE            128
#define APP_WIFI_AT_RESPONSE_SIZE   224
#define APP_VERSION                 "2026-10-29-r36"

/* ===================== 舵机通道 ===================== */
#define APP_SERVO_COUNT             6
#define APP_POSITION_CHANNEL_COUNT  5
#define APP_CONTINUOUS_CHANNEL      5
#define APP_SENSOR_COUNT            5
#define APP_GLOVE_ALL_BENT_ON       70

/* ===================== 5号360°连续舵机 ===================== */
#define APP_SPEED_STOP_CMD          90
#define APP_SPEED_STOP_DEFAULT_US   1740
#define APP_SPEED_DEADBAND_DEG      2
#define APP_TRIM_MIN_US             1000
#define APP_TRIM_MAX_US             2200

/* ===================== 位置舵机缓动 ===================== */
#define APP_POSITION_FAST_STEP      6
#define APP_POSITION_SLOW_STEP      2
#define APP_POSITION_SLOW_ZONE      20

/* ===================== 位置舵机演示 ===================== */
#define APP_DEMO_INTERVAL_MS        5
#define APP_DEMO_PAUSE_MS           100

/* ===================== 连续舵机演示 ===================== */
#define APP_DEMO_CH5_SPEED_A        120
#define APP_DEMO_CH5_SPEED_B        60
#define APP_DEMO_CH5_SPEED_A_MS     200
#define APP_DEMO_CH5_SPEED_B_MS     400

/* S3触发的一次性六号舵机动作，保持左:右:左=1:2:1但缩短总时长 */
#define APP_CH5_ONCE_LEFT_MS        200
#define APP_CH5_ONCE_RIGHT_MS       400

#endif
