#include "stm32f10x.h"
#include "usart1.h"
#include "usart2.h"
#include "delay.h"
#include "app_config.h"
#include <string.h>
#include <stdio.h>

#define ESP_RESPONSE_SIZE  256
#define ESP_CONNECTION_ID  0
#define ESP_TX_QUEUE_SIZE  8
#define ESP_TX_TEXT_SIZE   64
#define ESP_PROMPT_TIMEOUT_MS 400
#define ESP_SEND_TIMEOUT_MS   800

static char s_usb_line[DEMO_SENSOR_LINE_SIZE];
static uint16_t s_usb_len;
static char s_esp_line[ESP_RESPONSE_SIZE];
static uint16_t s_esp_len;
static uint8_t s_wifi_connected;
static uint8_t s_all_bent_latched;
static uint8_t s_s3_full_latched;
static uint32_t s_last_client_probe_ms;
static uint32_t s_last_display_ms;
static uint32_t s_last_adc_display_ms;
static uint16_t s_filtered_adc[DEMO_SENSOR_COUNT];
static uint16_t s_filter_history[DEMO_SENSOR_COUNT][5];
static uint8_t s_filter_index;
static uint8_t s_filter_count;
static uint8_t s_stable_bends[DEMO_SENSOR_COUNT];
static uint8_t s_pending_bends[DEMO_SENSOR_COUNT];
static uint8_t s_pending_counts[DEMO_SENSOR_COUNT];
static uint8_t s_stable_bend_valid;
static uint8_t s_esp_ready;
static uint16_t s_wifi_retry_delay_ms = DEMO_WIFI_RETRY_MS;
static uint32_t s_last_wifi_retry_ms;

typedef enum
{
    ESP_TX_IDLE = 0,
    ESP_TX_WAIT_PROMPT,
    ESP_TX_WAIT_SEND_OK
} EspTxState;

typedef struct
{
    char text[ESP_TX_TEXT_SIZE];
    uint8_t verbose;
    uint8_t probe;
} EspTxItem;

static EspTxItem s_tx_queue[ESP_TX_QUEUE_SIZE];
static uint8_t s_tx_head;
static uint8_t s_tx_tail;
static EspTxState s_tx_state;
static uint32_t s_tx_start_ms;

typedef struct
{
    uint16_t adc_open;
    uint16_t adc_half;
    uint16_t adc_bent;
} FlexCalibration;

/* 默认三点标定值，弯曲后ADC依次减小 */
static const FlexCalibration s_flex_cal_default[DEMO_SENSOR_COUNT] =
{
    {127, 114, 100},
    {1769, 1732, 1700},
    {1596, 1557, 1535},
    {1735, 1671, 1638},
    {130, 117, 91}
};
static FlexCalibration s_flex_cal[DEMO_SENSOR_COUNT];

static void USB_PrintLine(const char *text)
{
    USART1_SendString(text);
    USART1_SendString("\r\n");
}

static void ESP_DrainRx(void)
{
    while(USART2_RxAvailable())
        (void)USART2_ReadByte();
}

static uint8_t ESP_WaitFor(const char *keyword, uint32_t timeout_ms)
{
    char response[ESP_RESPONSE_SIZE];
    uint16_t length = 0;
    uint32_t start_ms = millis();
    uint8_t byte;

    response[0] = '\0';

    while((uint32_t)(millis() - start_ms) < timeout_ms)
    {
        while(USART2_RxAvailable())
        {
            byte = USART2_ReadByte();
            if(length < (ESP_RESPONSE_SIZE - 1))
            {
                response[length++] = (char)byte;
                response[length] = '\0';
            }
        }

        if(strstr(response, keyword) != 0)
            return 1;

        if(strstr(response, "ERROR") != 0 ||
           strstr(response, "FAIL") != 0)
            return 0;

        Delay_ms(1);
    }

    return 0;
}

static uint8_t ESP_SendCommand(const char *command,
                               const char *expect,
                               uint32_t timeout_ms)
{
    ESP_DrainRx();
    USART2_SendString(command);
    USART2_SendString("\r\n");
    return ESP_WaitFor(expect, timeout_ms);
}

static void ESP_ClearTxQueue(void)
{
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_state = ESP_TX_IDLE;
    s_tx_start_ms = 0;
    s_esp_len = 0;
}

static void ESP_ForceCommandMode(void)
{
    /*
     * ESP8266可能在CIPSEND提示符后等待数据，或仍处在透传模式。
     * 按要求先保持静默，再发送不带回车的+++，随后关闭旧TCP连接。
     */
    Delay_ms(1100);
    ESP_DrainRx();
    USART2_SendString("+++");
    Delay_ms(1100);
    ESP_DrainRx();
    (void)ESP_SendCommand("AT+CIPCLOSE=0", "OK", 400);
    ESP_DrainRx();
}

static void ESP_Config(uint8_t quick_retry)
{
    uint8_t ready = 1;
    uint8_t at_ok = 0;
    uint8_t attempt;
    uint8_t max_attempts = quick_retry ? 1 : 3;

    s_esp_ready = 0;
    s_wifi_connected = 0;
    s_last_client_probe_ms = 0;
    ESP_ClearTxQueue();
    USB_PrintLine("Configuring ESP8266...");
    ESP_ForceCommandMode();

    /* 模块上电可能比STM32慢，AT握手最多重试三次 */
    for(attempt = 0; attempt < max_attempts; attempt++)
    {
        if(ESP_SendCommand("AT", "OK", 800))
        {
            at_ok = 1;
            break;
        }
        Delay_ms(150);
    }

    if(!at_ok)
    {
        USB_PrintLine("[ESP] AT no response");
        USB_PrintLine("continue without WiFi, local ADC remains active");
        s_wifi_retry_delay_ms = DEMO_WIFI_RETRY_MS;
        s_last_wifi_retry_ms = millis();
        return;
    }

    (void)ESP_SendCommand("ATE0", "OK", 500);

    if(!ESP_SendCommand("AT+CWMODE=2", "OK", 1000))
    {
        USB_PrintLine("[ESP] CWMODE failed");
        ready = 0;
    }

    if(!ESP_SendCommand("AT+CWSAP=\"" DEMO_WIFI_AP_SSID "\",\""
                        DEMO_WIFI_AP_PASSWORD "\",5,3",
                        "OK", 2000))
    {
        USB_PrintLine("[ESP] CWSAP failed");
        ready = 0;
    }

    /* 部分AT固件不支持CIPMODE=0，服务器模式不依赖该命令 */
    (void)ESP_SendCommand("AT+CIPMODE=0", "OK", 500);

    if(!ESP_SendCommand("AT+CIPMUX=1", "OK", 1000))
    {
        USB_PrintLine("[ESP] CIPMUX failed");
        ready = 0;
    }

    if(!ESP_SendCommand("AT+CIPSERVER=1," DEMO_WIFI_TCP_PORT_TEXT,
                        "OK", 1500))
    {
        USB_PrintLine("[ESP] CIPSERVER failed");
        ready = 0;
    }

    s_esp_ready = ready;
    if(ready)
    {
        USB_PrintLine("ESP8266 TCP server ready");
        s_wifi_retry_delay_ms = DEMO_WIFI_RETRY_MS;
    }
    else
    {
        USB_PrintLine("ESP8266 server config failed");
        s_wifi_retry_delay_ms = DEMO_WIFI_RETRY_MS;
    }
    s_last_wifi_retry_ms = millis();
}

static void ESP_ScheduleRecovery(void)
{
    s_wifi_connected = 0;
    s_esp_ready = 0;
    s_last_client_probe_ms = 0;
    s_wifi_retry_delay_ms = DEMO_WIFI_RECOVER_MS;
    s_last_wifi_retry_ms = millis();
}

static void ESP_FallbackWired(const char *text)
{
    USART1_SendString(text);
    USART1_SendString("\r\n");
}

static void ESP_StartNextTx(void)
{
    char command[40];
    EspTxItem *item;
    uint16_t length;

    if(s_tx_state != ESP_TX_IDLE || s_tx_head == s_tx_tail)
        return;

    item = &s_tx_queue[s_tx_tail];
    length = (uint16_t)(strlen(item->text) + 2);
    snprintf(command, sizeof(command),
             "AT+CIPSEND=%u,%u", ESP_CONNECTION_ID,
             (unsigned int)length);

    s_esp_len = 0;
    s_tx_state = ESP_TX_WAIT_PROMPT;
    s_tx_start_ms = millis();
    USART2_SendString(command);
    USART2_SendString("\r\n");
}

static void ESP_FinishTx(uint8_t success)
{
    EspTxItem *item = &s_tx_queue[s_tx_tail];
    char display[ESP_TX_TEXT_SIZE + 24];

    if(success)
    {
        if(item->probe)
        {
            s_wifi_connected = 1;
            s_last_client_probe_ms = 0;
            USB_PrintLine("[WIFI] client detected, send OK");
        }

        if(item->verbose)
        {
            snprintf(display, sizeof(display),
                     "[WIFI TX OK] %s", item->text);
            USB_PrintLine(display);
        }

        s_tx_tail = (uint8_t)((s_tx_tail + 1) % ESP_TX_QUEUE_SIZE);
        s_tx_state = ESP_TX_IDLE;
        s_tx_start_ms = 0;
        s_esp_len = 0;
        ESP_StartNextTx();
        return;
    }

    if(item->probe)
    {
        ESP_ClearTxQueue();
        return;
    }

    /*
     * 已发送的数据不能静默丢弃，失败时用有线链路补发当前帧，
     * 随后清空过期队列并重建ESP8266，保证回正数据不会滞后。
     */
    ESP_FallbackWired(item->text);
    if(item->verbose)
    {
        USB_PrintLine("[WIFI TX FAIL] recovery scheduled");
        snprintf(display, sizeof(display),
                 "[WIRED TX] %s", item->text);
        USB_PrintLine(display);
    }
    ESP_ClearTxQueue();
    ESP_ScheduleRecovery();
}

static void ESP_PollTx(void)
{
    uint32_t timeout_ms;

    if(s_tx_state == ESP_TX_IDLE)
    {
        ESP_StartNextTx();
        return;
    }

    timeout_ms = (s_tx_state == ESP_TX_WAIT_PROMPT) ?
                 ESP_PROMPT_TIMEOUT_MS : ESP_SEND_TIMEOUT_MS;
    if((uint32_t)(millis() - s_tx_start_ms) >= timeout_ms)
        ESP_FinishTx(0);
}

static uint8_t ESP_QueueText(const char *text,
                             uint8_t verbose,
                             uint8_t probe)
{
    uint8_t next;
    EspTxItem *item;

    if(text == 0 || *text == '\0')
        return 0;

    next = (uint8_t)((s_tx_head + 1) % ESP_TX_QUEUE_SIZE);
    if(next == s_tx_tail)
        return 0;

    item = &s_tx_queue[s_tx_head];
    strncpy(item->text, text, ESP_TX_TEXT_SIZE - 1);
    item->text[ESP_TX_TEXT_SIZE - 1] = '\0';
    item->verbose = verbose;
    item->probe = probe;
    s_tx_head = next;
    ESP_StartNextTx();
    return 1;
}

/* 五路传感器：PA0、PA1、PA4、PA5、PA6，保留PA2/PA3给USART2 */
static const uint8_t s_flex_channels[DEMO_SENSOR_COUNT] =
{
    ADC_Channel_0,
    ADC_Channel_1,
    ADC_Channel_4,
    ADC_Channel_5,
    ADC_Channel_6
};

static void FlexAdc_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |
                    GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

static uint16_t FlexAdc_Read(uint8_t channel)
{
    uint16_t samples[3];
    uint16_t temp;
    uint8_t pass;
    uint8_t i;
    uint8_t j;

    ADC_RegularChannelConfig(ADC1, channel, 1,
                             ADC_SampleTime_239Cycles5);

    /*
     * 连续转换三次并取中值，可丢弃通道切换后的首个受采样电容影响的
     * 样本，同时抑制高阻弯曲传感器上的单点尖峰。
     */
    for(pass = 0; pass < 3; pass++)
    {
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        samples[pass] = ADC_GetConversionValue(ADC1);
    }

    for(i = 1; i < 3; i++)
    {
        temp = samples[i];
        j = i;
        while(j > 0 && samples[j - 1] > temp)
        {
            samples[j] = samples[j - 1];
            j--;
        }
        samples[j] = temp;
    }

    return samples[1];
}

static void FlexAdc_ReadAll(unsigned int values[DEMO_SENSOR_COUNT])
{
    uint8_t i;

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
        values[i] = FlexAdc_Read(s_flex_channels[i]);
}

static uint8_t ParseGloveRaw(const char *line,
                             unsigned int values[DEMO_SENSOR_COUNT])
{
    const char *raw = strstr(line, "RAW");
    const char *sensors;
    uint8_t i;

    if(raw != 0)
    {
        if(sscanf(raw,
                  "RAW S1:%u S2:%u S3:%u S4:%u S5:%u",
                  &values[0], &values[1], &values[2],
                  &values[3], &values[4]) == DEMO_SENSOR_COUNT)
        {
            for(i = 0; i < DEMO_SENSOR_COUNT; i++)
            {
                if(values[i] > 4095U)
                    return 0;
            }
            return 1;
        }
    }

    /* 兼容不带RAW前缀的五路文本 */
    sensors = strstr(line, "S1:");
    if(sensors == 0)
        return 0;

    if(sscanf(sensors,
              "S1:%u S2:%u S3:%u S4:%u S5:%u",
              &values[0], &values[1], &values[2],
              &values[3], &values[4]) != DEMO_SENSOR_COUNT)
        return 0;

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        if(values[i] > 4095U)
            return 0;
    }
    return 1;
}

static uint16_t MedianFive(const uint16_t input[5], uint8_t count)
{
    uint16_t sorted[5];
    uint16_t temp;
    uint8_t i;
    uint8_t j;

    for(i = 0; i < count; i++)
        sorted[i] = input[i];

    for(i = 1; i < count; i++)
    {
        temp = sorted[i];
        j = i;
        while(j > 0 && sorted[j - 1] > temp)
        {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = temp;
    }

    return sorted[count / 2];
}

static void FilterFlexReadings(
    const unsigned int raw[DEMO_SENSOR_COUNT])
{
    uint8_t i;

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        s_filter_history[i][s_filter_index] = (uint16_t)raw[i];
        s_filtered_adc[i] = (uint16_t)raw[i];
    }

    s_filter_index++;
    if(s_filter_index >= 5)
        s_filter_index = 0;

    if(s_filter_count < 5)
        s_filter_count++;

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        s_filtered_adc[i] = MedianFive(
            s_filter_history[i], s_filter_count);
    }
}

static void CaptureOpenBaseline(void)
{
    char display[128];
    FlexCalibration *cal;
    const FlexCalibration *fallback;
    uint16_t span;
    uint8_t i;

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        cal = &s_flex_cal[i];
        fallback = &s_flex_cal_default[i];
        cal->adc_bent = fallback->adc_bent;

        /*
         * 每颗弯曲传感器和每路ADC的静态值都有偏差，直接使用固定
         * “张开值”会把正常伸直误算成弯曲。这里以本次上电稳定读数
         * 作为张开基线，弯曲端点仍沿用已经测好的默认值。
         */
        if(s_filtered_adc[i] > (uint16_t)(fallback->adc_bent + 10U))
        {
            cal->adc_open = s_filtered_adc[i];
            span = (uint16_t)(cal->adc_open - cal->adc_bent);
            cal->adc_half = (uint16_t)(cal->adc_open - span / 2U);
        }
        else
        {
            cal->adc_open = fallback->adc_open;
            cal->adc_half = fallback->adc_half;
        }
    }

    s_stable_bend_valid = 0;
    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        s_stable_bends[i] = 0;
        s_pending_bends[i] = 0;
        s_pending_counts[i] = 0;
    }

    snprintf(display, sizeof(display),
             "[CAL] open S1=%u S2=%u S3=%u S4=%u S5=%u",
             (unsigned int)s_flex_cal[0].adc_open,
             (unsigned int)s_flex_cal[1].adc_open,
             (unsigned int)s_flex_cal[2].adc_open,
             (unsigned int)s_flex_cal[3].adc_open,
             (unsigned int)s_flex_cal[4].adc_open);
    USB_PrintLine(display);
}

static uint8_t CalibrateBend(uint8_t sensor, uint16_t adc)
{
    const FlexCalibration *cal = &s_flex_cal[sensor];
    uint32_t value;

    if(adc >= cal->adc_open)
        return 0;
    if(adc <= cal->adc_bent)
        return 100;

    if(adc >= cal->adc_half)
    {
        if(cal->adc_open <= cal->adc_half)
            return 50;
        value = (uint32_t)(cal->adc_open - adc) * 50U;
        value /= (uint32_t)(cal->adc_open - cal->adc_half);
        return (uint8_t)value;
    }

    if(cal->adc_half <= cal->adc_bent)
        return 50;
    value = 50U + (uint32_t)(cal->adc_half - adc) * 50U /
            (uint32_t)(cal->adc_half - cal->adc_bent);
    if(value > 100U)
        value = 100U;
    return (uint8_t)value;
}

static uint8_t CalibrateBendS2(uint16_t adc)
{
    const FlexCalibration *cal = &s_flex_cal[1];
    uint16_t drop;
    uint32_t value;

    if(cal->adc_open <= DEMO_S2_OPEN_DEADBAND ||
       DEMO_S2_FULL_SCALE_ADC <= DEMO_S2_OPEN_DEADBAND)
        return 0;

    if(adc >= (uint16_t)(cal->adc_open - DEMO_S2_OPEN_DEADBAND))
        return 0;

    drop = (uint16_t)(cal->adc_open -
                      DEMO_S2_OPEN_DEADBAND - adc);
    value = (uint32_t)drop * 100U /
            (uint32_t)(DEMO_S2_FULL_SCALE_ADC -
                       DEMO_S2_OPEN_DEADBAND);
    if(value > 100U)
        value = 100U;

    value = (value * DEMO_S2_GAIN_PERCENT + 50U) / 100U;
    if(value > 100U)
        value = 100U;
    return (uint8_t)value;
}

static void CalibrateFlexReadings(
    const uint16_t raw[DEMO_SENSOR_COUNT],
    uint8_t bends[DEMO_SENSOR_COUNT])
{
    uint8_t i;
    uint8_t target;
    uint8_t delta;
    uint8_t step;

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        /*
         * 0或满量程通常表示传感器断线、短路或ADC输入悬空，
         * 不能把它换算成100%后让机械手误动作。
         */
        if(raw[i] <= DEMO_ADC_VALID_MIN ||
           raw[i] >= DEMO_ADC_VALID_MAX)
        {
            bends[i] = s_stable_bend_valid ? s_stable_bends[i] : 0;
            s_pending_counts[i] = 0;
            continue;
        }

        target = (i == 1) ?
                 CalibrateBendS2(raw[i]) :
                 CalibrateBend(i, raw[i]);

        if(!s_stable_bend_valid)
        {
            s_stable_bends[i] = target;
            s_pending_bends[i] = target;
            s_pending_counts[i] = 0;
            bends[i] = target;
            continue;
        }

        if(target > s_stable_bends[i])
            delta = (uint8_t)(target - s_stable_bends[i]);
        else
            delta = (uint8_t)(s_stable_bends[i] - target);

        /*
         * 较大变化必须连续出现两次才接受，避免悬空输入或电源干扰
         * 让机械手突然误动作；确认后再用低通限制变化速度。
         */
        if(delta >= 20)
        {
            if(target == s_pending_bends[i])
            {
                if(s_pending_counts[i] < 255)
                    s_pending_counts[i]++;
            }
            else
            {
                s_pending_bends[i] = target;
                s_pending_counts[i] = 1;
            }

            if(s_pending_counts[i] < DEMO_BEND_CONFIRM)
            {
                bends[i] = s_stable_bends[i];
                continue;
            }
        }
        else
        {
            s_pending_bends[i] = target;
            s_pending_counts[i] = 0;
        }

        if(delta <= DEMO_BEND_DEADBAND)
        {
            bends[i] = s_stable_bends[i];
            continue;
        }

        if(target < s_stable_bends[i])
            step = (uint8_t)((delta + 1U) / 2U);
        else
            step = (uint8_t)((delta + 3U) / 4U);
        if(step == 0)
            step = 1;

        if(target < s_stable_bends[i])
            s_stable_bends[i] =
                (uint8_t)(s_stable_bends[i] - step);
        else
            s_stable_bends[i] =
                (uint8_t)(s_stable_bends[i] + step);
        bends[i] = s_stable_bends[i];
    }

    s_stable_bend_valid = 1;
}

static void PrintAdcStatus(void)
{
    char display[128];

    if(s_last_adc_display_ms != 0 &&
       (uint32_t)(millis() - s_last_adc_display_ms) < 1000U)
        return;

    s_last_adc_display_ms = millis();
    snprintf(display, sizeof(display),
             "[ADC] S1=%u S2=%u S3=%u S4=%u S5=%u",
             (unsigned int)s_filtered_adc[0],
             (unsigned int)s_filtered_adc[1],
             (unsigned int)s_filtered_adc[2],
             (unsigned int)s_filtered_adc[3],
             (unsigned int)s_filtered_adc[4]);
    USB_PrintLine(display);
}

static void FormatWifiPayload(const uint8_t bends[DEMO_SENSOR_COUNT],
                              char *payload,
                              uint16_t payload_size)
{
    snprintf(payload, payload_size,
             "GLOVE,%u,%u,%u,%u,%u",
             (unsigned int)bends[0], (unsigned int)bends[1],
             (unsigned int)bends[2], (unsigned int)bends[3],
             (unsigned int)bends[4]);
}

static uint8_t SendWifiText(const char *text, uint8_t verbose)
{
    char display[DEMO_WIFI_LINE_SIZE + 24];

    if(!s_wifi_connected)
    {
        /*
         * WiFi不可用时通过USART1有线发送同一条控制命令。
         * 接线：demo PA9 TX -> 机械手STM32F407 PA10 RX，双方共地。
         */
        ESP_FallbackWired(text);

        /*
         * TCP CONNECT通知可能因模块提前连接或上电顺序而漏掉。
         * 每秒做一次非阻塞CIPSEND探测；没有客户端时只得到ERROR，
         * 不会占住采样和其他串口处理。
         */
        if(s_esp_ready &&
           (s_last_client_probe_ms == 0 ||
            (uint32_t)(millis() - s_last_client_probe_ms) >= 1000U))
        {
            s_last_client_probe_ms = millis();
            (void)ESP_QueueText(text, 0, 1);
        }

        if(verbose)
        {
            snprintf(display, sizeof(display), "[WIRED TX] %s", text);
            USB_PrintLine(display);
        }
        return 1;
    }

    if(ESP_QueueText(text, verbose, 0))
        return 1;

    /* 队列满时不能等待，直接发送最新有线数据来压低控制延迟 */
    ESP_FallbackWired(text);
    if(verbose)
    {
        USB_PrintLine("[WIFI QUEUE FULL] using wired fallback");
        snprintf(display, sizeof(display), "[WIRED TX] %s", text);
        USB_PrintLine(display);
    }
    return 1;
}

static void ProcessBendValues(const uint8_t bends[DEMO_SENSOR_COUNT])
{
    char payload[DEMO_WIFI_LINE_SIZE];
    char display[DEMO_WIFI_LINE_SIZE + 24];
    uint8_t i;
    uint8_t all_bent = 1;
    uint8_t s3_full;
    uint8_t verbose;

    verbose = (s_last_display_ms == 0 ||
               (uint32_t)(millis() - s_last_display_ms) >= DEMO_DISPLAY_MS);
    if(verbose)
    {
        s_last_display_ms = millis();
        snprintf(display, sizeof(display),
                 "[BEND] S1=%u%% S2=%u%% S3=%u%% S4=%u%% S5=%u%%",
                 (unsigned int)bends[0], (unsigned int)bends[1],
                 (unsigned int)bends[2], (unsigned int)bends[3],
                 (unsigned int)bends[4]);
        USB_PrintLine(display);
    }

    for(i = 0; i < DEMO_SENSOR_COUNT; i++)
    {
        if(bends[i] < DEMO_ALL_BENT_ON)
            all_bent = 0;
    }

    s3_full = (bends[2] >= DEMO_S3_TRIGGER_ON);
    if(s3_full)
    {
        /*
         * S3达到触发阈值时，机械手CH5执行一次：
         * 左1秒 -> 右2秒 -> 左1秒 -> 停止。
         */
        if(!s_s3_full_latched)
        {
            if(SendWifiText("CH5_ONCE", verbose))
            {
                s_s3_full_latched = 1;
                s_all_bent_latched = 1;
                USB_PrintLine("[S3 TRIGGER] CH5_ONCE sent");
            }
        }
    }
    else if(bends[2] <= DEMO_S3_TRIGGER_OFF)
    {
        s_s3_full_latched = 0;
    }

    if(all_bent)
    {
        if(!s3_full && !s_all_bent_latched)
        {
            if(SendWifiText("DEMO_ONCE", verbose))
                s_all_bent_latched = 1;
        }
    }
    else
    {
        for(i = 0; i < DEMO_SENSOR_COUNT; i++)
        {
            if(bends[i] <= DEMO_ALL_BENT_OFF)
                s_all_bent_latched = 0;
        }
    }

    /*
     * 演示触发后仍持续发送实时角度。
     * 机械手在五路不再全弯时会立即退出演示并恢复单指控制。
     */
    FormatWifiPayload(bends, payload, sizeof(payload));
    (void)SendWifiText(payload, verbose);
}

static void HandleUsbLine(char *line)
{
    unsigned int values[DEMO_SENSOR_COUNT];
    uint16_t raw[DEMO_SENSOR_COUNT];
    uint8_t bends[DEMO_SENSOR_COUNT];
    char display[DEMO_SENSOR_LINE_SIZE + 16];
    uint8_t i;

    snprintf(display, sizeof(display), "[USB RX] %s", line);
    USB_PrintLine(display);

    if(strcmp(line, "CAL") == 0 || strcmp(line, "cal") == 0)
    {
        CaptureOpenBaseline();
        CalibrateFlexReadings(s_filtered_adc, bends);
        ProcessBendValues(bends);
        return;
    }

    if(ParseGloveRaw(line, values))
    {
        for(i = 0; i < DEMO_SENSOR_COUNT; i++)
            raw[i] = (uint16_t)values[i];
        CalibrateFlexReadings(raw, bends);
        ProcessBendValues(bends);
    }
}

static void PollUsbData(void)
{
    uint8_t byte;

    while(USART1_RxAvailable())
    {
        byte = USART1_ReadByte();
        if(byte == '\r' || byte == '\n')
        {
            if(s_usb_len > 0)
            {
                s_usb_line[s_usb_len] = '\0';
                HandleUsbLine(s_usb_line);
                s_usb_len = 0;
            }
        }
        else if(s_usb_len < (DEMO_SENSOR_LINE_SIZE - 1))
        {
            s_usb_line[s_usb_len++] = (char)byte;
        }
        else
        {
            s_usb_len = 0;
        }
    }
}

static void HandleEspLine(char *line)
{
    char display[ESP_RESPONSE_SIZE + 24];

    if(strcmp(line, "ready") == 0 ||
       strstr(line, "WIFI DISCONNECT") != 0)
    {
        ESP_ClearTxQueue();
        ESP_ScheduleRecovery();
        USB_PrintLine("[WIFI] module restarted, recovery scheduled");
        return;
    }

    if(strstr(line, "0,CONNECT") != 0)
    {
        s_wifi_connected = 1;
        USB_PrintLine("[WIFI] TCP client connected");
        return;
    }

    if(strstr(line, "0,CLOSED") != 0)
    {
        ESP_ClearTxQueue();
        s_wifi_connected = 0;
        USB_PrintLine("[WIFI] TCP client disconnected");
        return;
    }

    if(strcmp(line, "OK") == 0)
        return;

    if(strstr(line, "+IPD") != 0)
    {
        snprintf(display, sizeof(display), "[WIFI RX] %s", line);
        USB_PrintLine(display);
    }
}

static uint8_t ESP_TxResponseFailed(const char *response)
{
    return strstr(response, "ERROR") != 0 ||
           strstr(response, "FAIL") != 0 ||
           strstr(response, "CLOSED") != 0 ||
           strstr(response, "busy") != 0;
}

static void ESP_HandleTxByte(uint8_t byte)
{
    EspTxItem *item;

    item = &s_tx_queue[s_tx_tail];

    if(s_tx_state == ESP_TX_WAIT_PROMPT && byte == '>')
    {
        USART2_SendString(item->text);
        USART2_SendString("\r\n");
        s_esp_len = 0;
        s_tx_state = ESP_TX_WAIT_SEND_OK;
        s_tx_start_ms = millis();
        return;
    }

    if(byte == '\r' || byte == '\n')
    {
        if(s_esp_len > 0)
        {
            s_esp_line[s_esp_len] = '\0';

            if(s_tx_state == ESP_TX_WAIT_SEND_OK &&
               strstr(s_esp_line, "SEND OK") != 0)
            {
                ESP_FinishTx(1);
                return;
            }

            if(ESP_TxResponseFailed(s_esp_line))
            {
                ESP_FinishTx(0);
                return;
            }
        }

        s_esp_len = 0;
        return;
    }

    if(s_esp_len < (ESP_RESPONSE_SIZE - 1))
        s_esp_line[s_esp_len++] = (char)byte;
}

static void PollEspData(void)
{
    uint8_t byte;

    while(USART2_RxAvailable())
    {
        byte = USART2_ReadByte();
        if(s_tx_state != ESP_TX_IDLE)
        {
            ESP_HandleTxByte(byte);
            continue;
        }

        if(byte == '\r' || byte == '\n')
        {
            if(s_esp_len > 0)
            {
                s_esp_line[s_esp_len] = '\0';
                HandleEspLine(s_esp_line);
                s_esp_len = 0;
            }
        }
        else if(s_esp_len < (ESP_RESPONSE_SIZE - 1))
        {
            s_esp_line[s_esp_len++] = (char)byte;
        }
        else
        {
            s_esp_len = 0;
        }
    }
}

int main(void)
{
    char version[80];
    unsigned int sensor_values[DEMO_SENSOR_COUNT];
    uint8_t bends[DEMO_SENSOR_COUNT];
    uint8_t warmup;
    uint32_t last_sample_ms;
    uint32_t last_send_ms;

    Delay_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(DEMO_USB_BAUD);

    /* 在初始化ESP8266之前直接发送，用于隔离USART1硬件问题 */
    USART1_SendString("\r\nUSART1 BOOT OK\r\n");

    USART2_Init(DEMO_WIFI_BAUD);
    FlexAdc_Init();

    snprintf(version, sizeof(version),
             "=== glove WiFi bridge %s ===", DEMO_VERSION);
    USB_PrintLine(version);
    USB_PrintLine("USART1=USB-TTL, USART2=ESP8266");
    USB_PrintLine("sampling PA0/PA1/PA4/PA5/PA6, WiFi update 100ms");

    /* ESP配置前先输出一次本地ADC值，便于确认传感器接线 */
    FlexAdc_ReadAll(sensor_values);
    FilterFlexReadings(sensor_values);
    PrintAdcStatus();

    /* 上电后先让传感器和电源稳定，再采集五路张开基线 */
    Delay_ms(500);
    for(warmup = 0; warmup < 20; warmup++)
    {
        FlexAdc_ReadAll(sensor_values);
        FilterFlexReadings(sensor_values);
        Delay_ms(10);
    }
    CaptureOpenBaseline();
    CalibrateFlexReadings(s_filtered_adc, bends);
    ProcessBendValues(bends);

    Delay_ms(200);
    ESP_Config(0);
    last_sample_ms = millis();
    last_send_ms = last_sample_ms;

    while(1)
    {
        PollUsbData();
        PollEspData();
        ESP_PollTx();

        /* WiFi初始化失败后周期重试，避免一次AT丢失导致整次上电都没有网络 */
        if(!s_esp_ready &&
           (uint32_t)(millis() - s_last_wifi_retry_ms) >=
           s_wifi_retry_delay_ms)
        {
            s_last_wifi_retry_ms = millis();
            USB_PrintLine("[ESP] retry WiFi setup");
            ESP_Config(1);
            last_sample_ms = millis();
            last_send_ms = last_sample_ms;
        }

        if((uint32_t)(millis() - last_sample_ms) >= DEMO_SENSOR_SAMPLE_MS)
        {
            last_sample_ms = millis();
            FlexAdc_ReadAll(sensor_values);
            FilterFlexReadings(sensor_values);
            PrintAdcStatus();
        }

        if((uint32_t)(millis() - last_send_ms) >= DEMO_WIFI_SEND_MS)
        {
            last_send_ms = millis();
            CalibrateFlexReadings(s_filtered_adc, bends);
            ProcessBendValues(bends);
        }

        Delay_ms(1);
    }
}
