#include "myhead.h"

static void App_PrintCommandInfo(void)
{
    Comm_SendLine(COMM_SOURCE_SERIAL,
                  "=== servo ready: CH0-4 position, CH5 continuous ===");
    Comm_SendLine(COMM_SOURCE_SERIAL, "firmware: " APP_VERSION);
    Comm_SendLine(COMM_SOURCE_SERIAL,
                  "CH0-4: angle | CH5: 0=reverse,90=stop,180=forward");
    Comm_SendLine(COMM_SOURCE_SERIAL,
                  "CH5 also accepts raw pulse: 500-2500us");
    Comm_SendLine(COMM_SOURCE_SERIAL,
                  "CH5 diagnostics: CAL5=1000-2000us, FINE5=1670-1810us, OFF5");
    Comm_SendLine(COMM_SOURCE_SERIAL,
                  "D=full finger demo, DEMO5=continuous servo demo only");
    Comm_SendLine(COMM_SOURCE_SERIAL,
                  "Bluetooth default 9600; use BTBAUD:115200 to change");
}

static void App_PrintWifiInfo(uint8_t wifi_ok)
{
    if(wifi_ok)
    {
        Comm_SendLine(COMM_SOURCE_SERIAL,
                      "WiFi client: " WIFI_STA_SSID " -> "
                      WIFI_SERVER_IP ":" WIFI_TCP_PORT_TEXT);
    }
    else
    {
        Comm_SendLine(COMM_SOURCE_SERIAL, "WiFi module not detected");
    }
}

int main(void)
{
    uint8_t wifi_ok;
    uint32_t bluetooth_rx_count;
    uint32_t bluetooth_announced_count = 0;
    uint16_t bluetooth_ready_tick = 0;

    /* 初始化硬件通信和PCA9685 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Comm_Init();
    init_usart1(APP_SERIAL_BAUD);
    Comm_InitBluetooth(APP_BLUETOOTH_BAUD);
    Comm_SendLine(COMM_SOURCE_BLUETOOTH,
                  "BT READY " APP_VERSION ", uart=9600");
    init_usart3(WIFI_UART_BAUD);
    PCA9685_Init();

    /* 初始化舵机状态：0～4回到原角度，5号停止 */
    Servo_ControlInit();

    App_PrintCommandInfo();
    wifi_ok = WiFi_Init();
    App_PrintWifiInfo(wifi_ok);

    while(1)
    {
        /* 蓝牙优先处理，避免手套高频有线日志让控制命令排队 */
        Command_ProcessInput(COMM_SOURCE_BLUETOOTH);
        Command_ProcessInput(COMM_SOURCE_SERIAL);
        Command_ProcessInput(COMM_SOURCE_WIFI);
        Command_UpdateCalibration(APP_TICK_MS);

        /* WiFi连接管理、位置缓动和演示动作 */
        WiFi_Poll(APP_TICK_MS);
        Servo_UpdatePositions();
        Servo_UpdateDemo();

        /*
         * 蓝牙模块可能在单片机启动后才连接，因此在尚未收到任何
         * 蓝牙字节时周期发送提示，帮助确认模块供电、TX和波特率。
         */
        bluetooth_rx_count =
            Comm_GetRxByteCount(COMM_SOURCE_BLUETOOTH);
        if(bluetooth_rx_count > 0 && bluetooth_announced_count == 0)
        {
            bluetooth_announced_count = bluetooth_rx_count;
            Comm_SendLine(COMM_SOURCE_BLUETOOTH,
                          "BT RX PATH OK, command bytes received");
        }
        if(bluetooth_rx_count == 0)
        {
            bluetooth_ready_tick =
                (uint16_t)(bluetooth_ready_tick + APP_TICK_MS);
            if(bluetooth_ready_tick >= 3000U)
            {
                bluetooth_ready_tick = 0;
                Comm_SendLine(COMM_SOURCE_BLUETOOTH,
                              "BT READY 9600, waiting command");
            }
        }

        delay_ms(APP_TICK_MS);
    }
}
