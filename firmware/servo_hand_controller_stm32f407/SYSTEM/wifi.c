#include "wifi.h"
#include "usart.h"
#include "comm.h"
#include "delay.h"
#include <string.h>

#define WIFI_RESPONSE_SIZE       512
#define WIFI_AT_QUEUE_SIZE       512
#define WIFI_JOIN_RETRY_MS       5000
#define WIFI_TCP_RETRY_MS        1500
#define WIFI_AT_TIMEOUT_MS       500
#define WIFI_COMMAND_TIMEOUT_MS  1200
#define WIFI_JOIN_TIMEOUT_MS     7000
#define WIFI_ESCAPE_GUARD_MS     1100
#define WIFI_ESCAPE_TIMEOUT_MS   1600
#define WIFI_RX_TIMEOUT_MS       2000

typedef enum
{
    WIFI_TASK_ESCAPE_GUARD = 0,
    WIFI_TASK_WAIT_ESCAPE,
    WIFI_TASK_ESCAPE_SETTLE,
    WIFI_TASK_WAIT_CLOSE,
    WIFI_TASK_IDLE,
    WIFI_TASK_WAIT_AT,
    WIFI_TASK_WAIT_ECHO,
    WIFI_TASK_WAIT_MODE,
    WIFI_TASK_WAIT_JOIN,
    WIFI_TASK_WAIT_MUX,
    WIFI_TASK_WAIT_CIPMODE,
    WIFI_TASK_WAIT_CONNECT,
    WIFI_TASK_WAIT_PROMPT
} WiFiTaskState;

static uint8_t s_module_ready;
static uint8_t s_network_ready;
static uint8_t s_transparent_mode;
static uint8_t s_suspended;
static uint32_t s_wifi_baud = WIFI_UART_BAUD;
static uint8_t s_retry_count;
static uint16_t s_retry_tick;
static WiFiTaskState s_task_state;
static uint16_t s_task_tick;
static uint16_t s_task_timeout;
static char s_response[WIFI_RESPONSE_SIZE];
static uint16_t s_response_length;
static uint8_t s_at_rx_queue[WIFI_AT_QUEUE_SIZE];
static volatile uint16_t s_at_rx_head;
static volatile uint16_t s_at_rx_tail;
static uint16_t s_payload_idle_ms;

void WiFi_RxByteFromISR(uint8_t byte)
{
    uint16_t next;

    next = (uint16_t)((s_at_rx_head + 1) % WIFI_AT_QUEUE_SIZE);
    if(next != s_at_rx_tail)
    {
        s_at_rx_queue[s_at_rx_head] = byte;
        s_at_rx_head = next;
    }
}

static uint8_t WiFi_AtRxAvailable(void)
{
    return (s_at_rx_head != s_at_rx_tail);
}

static uint8_t WiFi_AtRxRead(void)
{
    uint8_t byte = s_at_rx_queue[s_at_rx_tail];
    s_at_rx_tail = (uint16_t)((s_at_rx_tail + 1) % WIFI_AT_QUEUE_SIZE);
    return byte;
}

static void WiFi_AtRxClear(void)
{
    s_at_rx_tail = s_at_rx_head;
}

static uint8_t WiFi_Contains(const char *text, const char *key)
{
    return strstr(text, key) != 0;
}

static void WiFi_StartTask(const char *command,
                           WiFiTaskState state,
                           uint16_t timeout_ms)
{
    WiFi_AtRxClear();
    s_response[0] = '\0';
    s_response_length = 0;
    s_task_tick = 0;
    s_task_timeout = timeout_ms;
    s_task_state = state;

    send3_str((char *)command);
    send3_str("\r\n");
}

static void WiFi_FinishTask(uint8_t success)
{
    s_task_state = WIFI_TASK_IDLE;
    s_task_tick = 0;
    s_retry_tick = WIFI_JOIN_RETRY_MS;
    WiFi_AtRxClear();

    if(!success)
    {
        s_retry_count++;
        if(s_retry_count >= 3)
        {
            s_retry_count = 0;
            Comm_SendLine(COMM_SOURCE_SERIAL,
                          s_network_ready ?
                          "wifi: waiting for demo TCP server" :
                          "wifi: WiFi setup retry failed");
        }
    }
}

static void WiFi_StartEscapeGuard(void)
{
    s_task_state = WIFI_TASK_ESCAPE_GUARD;
    s_task_tick = 0;
    s_task_timeout = WIFI_ESCAPE_GUARD_MS;
    s_response[0] = '\0';
    s_response_length = 0;
    WiFi_AtRxClear();
}

static void WiFi_RecoverLink(const char *reason)
{
    s_transparent_mode = 0;
    s_network_ready = 0;
    s_module_ready = 0;
    s_task_state = WIFI_TASK_IDLE;
    s_task_tick = 0;
    s_retry_count = 0;
    s_payload_idle_ms = 0;
    Comm_Clear(COMM_SOURCE_WIFI);
    Comm_SendLine(COMM_SOURCE_SERIAL, reason);
    WiFi_StartEscapeGuard();
}

uint8_t WiFi_Init(void)
{
    s_suspended = 0;
    s_module_ready = 0;
    s_network_ready = 0;
    s_transparent_mode = 0;
    s_retry_count = 0;
    s_retry_tick = 0;
    s_task_state = WIFI_TASK_ESCAPE_GUARD;
    s_task_tick = 0;
    s_task_timeout = 0;
    s_response_length = 0;
    s_payload_idle_ms = 0;

    s_at_rx_head = 0;
    s_at_rx_tail = 0;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    Comm_Clear(COMM_SOURCE_WIFI);

    /*
     * 单片机单独复位时ESP8266可能仍停在CIPSEND透传模式。
     * 先静默1.1秒，再发送不带回车的+++退出透传。
     */
    WiFi_StartEscapeGuard();
    return 1;
}

void WiFi_Suspend(void)
{
    uint8_t was_transparent;

    /*
     * 先退出透传和无线连接，再停止软件侧处理。
     * 仅清标志不能让ESP8266停止无线收发，并线使用时仍会干扰蓝牙。
     */
    was_transparent = s_transparent_mode;
    s_suspended = 1;
    s_transparent_mode = 0;
    s_network_ready = 0;
    s_module_ready = 0;
    s_task_state = WIFI_TASK_IDLE;
    s_task_tick = 0;
    s_retry_tick = 0;
    s_retry_count = 0;
    s_payload_idle_ms = 0;
    WiFi_AtRxClear();
    Comm_Clear(COMM_SOURCE_WIFI);

    if(was_transparent)
    {
        /* 透明模式下必须先静默1.1秒，再发送不带回车的+++ */
        delay_ms(WIFI_ESCAPE_GUARD_MS);
        WiFi_AtRxClear();
        send3_str("+++");
        delay_ms(WIFI_ESCAPE_TIMEOUT_MS);
        WiFi_AtRxClear();
    }

    send3_str("AT+CIPCLOSE\r\n");
    delay_ms(300);
    send3_str("AT+CWQAP\r\n");
    delay_ms(600);
    WiFi_AtRxClear();
    Comm_Clear(COMM_SOURCE_WIFI);
}

void WiFi_Resume(void)
{
    (void)WiFi_Init();
}

uint8_t WiFi_SetBaud(uint32_t baud)
{
    if(baud != 4800 && baud != 9600 && baud != 19200 &&
       baud != 38400 && baud != 57600 && baud != 74880 &&
       baud != 115200 && baud != 230400)
        return 0;

    s_wifi_baud = baud;
    init_usart3(baud);
    Comm_Clear(COMM_SOURCE_WIFI);
    return WiFi_Init();
}

uint32_t WiFi_GetBaud(void)
{
    return s_wifi_baud;
}

uint8_t WiFi_ATCommand(const char *command,
                       char *response,
                       uint16_t response_size,
                       uint16_t timeout_ms)
{
    uint16_t elapsed = 0;
    uint16_t length = 0;
    uint16_t ch;
    uint8_t complete = 0;
    uint8_t was_transparent;

    if(command == 0 || response == 0 || response_size == 0)
        return 0;

    /* 暂停后台连接流程，独占USART3进行AT诊断 */
    was_transparent = s_transparent_mode;
    s_transparent_mode = 0;
    s_task_state = WIFI_TASK_IDLE;
    s_task_tick = 0;
    response[0] = '\0';

    /*
     * 透传模式下发AT会被当作TCP数据发送到电脑端。
     * WIFIAT诊断前先按1.1秒静默、+++、再静默的顺序退出。
     */
    if(was_transparent)
    {
        delay_ms(WIFI_ESCAPE_GUARD_MS);
        WiFi_AtRxClear();
        send3_str("+++");
        delay_ms(WIFI_ESCAPE_TIMEOUT_MS);
        WiFi_AtRxClear();
        send3_str("AT+CIPCLOSE\r\n");
        delay_ms(300);
        WiFi_AtRxClear();
    }

    WiFi_AtRxClear();
    send3_str((char *)command);
    send3_str("\r\n");

    while(elapsed < timeout_ms)
    {
        while(WiFi_AtRxAvailable())
        {
            ch = WiFi_AtRxRead();
            if(length < (response_size - 1))
            {
                if(ch == '\r' || ch == '\n' ||
                   (ch >= 32 && ch <= 126))
                    response[length++] = (char)ch;
                else
                    response[length++] = '?';
                response[length] = '\0';
            }
        }

        if(WiFi_Contains(response, "OK") ||
           WiFi_Contains(response, "ERROR") ||
           WiFi_Contains(response, "FAIL") ||
           WiFi_Contains(response, ">"))
        {
            complete = 1;
            break;
        }

        delay_ms(1);
        elapsed++;
    }

    WiFi_AtRxClear();
    Comm_Clear(COMM_SOURCE_WIFI);
    s_retry_tick = WIFI_JOIN_RETRY_MS;

    if(WiFi_Contains(response, "OK"))
        s_module_ready = 1;

    return complete;
}

void WiFi_Poll(uint16_t elapsed_ms)
{
    uint16_t ch;

    if(s_suspended)
        return;

    if(s_transparent_mode)
    {
        if(s_payload_idle_ms < WIFI_RX_TIMEOUT_MS)
        {
            uint32_t next_idle =
                (uint32_t)s_payload_idle_ms + elapsed_ms;
            s_payload_idle_ms = (next_idle >= WIFI_RX_TIMEOUT_MS) ?
                                WIFI_RX_TIMEOUT_MS : (uint16_t)next_idle;
        }

        if(s_payload_idle_ms >= WIFI_RX_TIMEOUT_MS)
            WiFi_RecoverLink("wifi: rx timeout, reconnecting");
        return;
    }

    if(s_task_state == WIFI_TASK_ESCAPE_GUARD)
    {
        s_task_tick = (uint16_t)(s_task_tick + elapsed_ms);
        if(s_task_tick < s_task_timeout)
            return;

        WiFi_AtRxClear();
        send3_str("+++");
        s_response[0] = '\0';
        s_response_length = 0;
        s_task_tick = 0;
        s_task_timeout = WIFI_ESCAPE_TIMEOUT_MS;
        s_task_state = WIFI_TASK_WAIT_ESCAPE;
        return;
    }

    if(s_task_state == WIFI_TASK_ESCAPE_SETTLE)
    {
        s_task_tick = (uint16_t)(s_task_tick + elapsed_ms);
        if(s_task_tick < s_task_timeout)
            return;

        /* +++前后都需要保持串口静默，避免退出透传失败 */
        WiFi_StartTask("AT+CIPCLOSE", WIFI_TASK_WAIT_CLOSE,
                       WIFI_COMMAND_TIMEOUT_MS);
        return;
    }

    if(s_task_state == WIFI_TASK_IDLE)
    {
        s_retry_tick = (uint16_t)(s_retry_tick + elapsed_ms);

        if(s_network_ready)
        {
            if(s_retry_tick < WIFI_TCP_RETRY_MS)
                return;
            WiFi_StartTask(
                "AT+CIPSTART=\"TCP\",\"" WIFI_SERVER_IP "\"," WIFI_TCP_PORT_TEXT,
                WIFI_TASK_WAIT_CONNECT,
                WIFI_COMMAND_TIMEOUT_MS);
        }
        else
        {
            if(s_retry_tick < WIFI_JOIN_RETRY_MS)
                return;
            /*
             * 上一次AT失败后ESP可能仍停留在透明模式。
             * 每次完整重试都先退出透传，避免反复把AT发进TCP连接。
             */
            WiFi_StartEscapeGuard();
        }
        return;
    }

    s_task_tick = (uint16_t)(s_task_tick + elapsed_ms);

    while(WiFi_AtRxAvailable())
    {
        ch = WiFi_AtRxRead();
        if(s_response_length < (WIFI_RESPONSE_SIZE - 1))
        {
            s_response[s_response_length++] = (char)ch;
            s_response[s_response_length] = '\0';
        }
    }

    if(s_task_state == WIFI_TASK_WAIT_ESCAPE)
    {
        if(WiFi_Contains(s_response, "OK") ||
           WiFi_Contains(s_response, "ERROR") ||
           WiFi_Contains(s_response, "FAIL") ||
           s_task_tick >= s_task_timeout)
        {
            s_response[0] = '\0';
            s_response_length = 0;
            s_task_tick = 0;
            s_task_timeout = WIFI_ESCAPE_GUARD_MS;
            s_task_state = WIFI_TASK_ESCAPE_SETTLE;
        }
        return;
    }

    if(s_task_state == WIFI_TASK_WAIT_CLOSE)
    {
        /*
         * 没有旧连接时CIPCLOSE通常返回ERROR，同样可以继续AT检测。
         */
        if(WiFi_Contains(s_response, "OK") ||
           WiFi_Contains(s_response, "ERROR") ||
           WiFi_Contains(s_response, "FAIL") ||
           s_task_tick >= s_task_timeout)
        {
            WiFi_StartTask("AT", WIFI_TASK_WAIT_AT, WIFI_AT_TIMEOUT_MS);
        }
        return;
    }

    /*
     * 连接已经存在时，AT+CIPSTART会返回ALREADY CONNECTED和ERROR。
     * 这种情况应继续执行CIPSEND进入透明模式，不能当作连接失败。
     */
    if((WiFi_Contains(s_response, "ERROR") ||
        WiFi_Contains(s_response, "FAIL")) &&
       !(s_task_state == WIFI_TASK_WAIT_CONNECT &&
         WiFi_Contains(s_response, "CONNECT")))
    {
        WiFi_FinishTask(0);
        return;
    }

    switch(s_task_state)
    {
        case WIFI_TASK_WAIT_AT:
            if(WiFi_Contains(s_response, "OK"))
            {
                s_module_ready = 1;
                WiFi_StartTask("ATE0", WIFI_TASK_WAIT_ECHO,
                               WIFI_COMMAND_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_ECHO:
            if(WiFi_Contains(s_response, "OK"))
            {
                WiFi_StartTask("AT+CWMODE=1", WIFI_TASK_WAIT_MODE,
                               WIFI_COMMAND_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_MODE:
            if(WiFi_Contains(s_response, "OK"))
            {
                WiFi_StartTask(
                    "AT+CWJAP=\"" WIFI_STA_SSID "\",\"" WIFI_STA_PASSWORD "\"",
                    WIFI_TASK_WAIT_JOIN,
                    WIFI_JOIN_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_JOIN:
            if(WiFi_Contains(s_response, "OK"))
            {
                s_network_ready = 1;
                WiFi_StartTask("AT+CIPMUX=0", WIFI_TASK_WAIT_MUX,
                               WIFI_COMMAND_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_MUX:
            if(WiFi_Contains(s_response, "OK"))
            {
                WiFi_StartTask("AT+CIPMODE=1", WIFI_TASK_WAIT_CIPMODE,
                               WIFI_COMMAND_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_CIPMODE:
            if(WiFi_Contains(s_response, "OK"))
            {
                Comm_SendLine(COMM_SOURCE_SERIAL,
                              "wifi: joined PYY_8266");
                WiFi_StartTask(
                    "AT+CIPSTART=\"TCP\",\"" WIFI_SERVER_IP "\"," WIFI_TCP_PORT_TEXT,
                    WIFI_TASK_WAIT_CONNECT,
                    WIFI_COMMAND_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_CONNECT:
            if(WiFi_Contains(s_response, "CONNECT"))
            {
                if(WiFi_Contains(s_response, "ALREADY"))
                {
                    Comm_SendLine(COMM_SOURCE_SERIAL,
                                  "wifi: existing TCP link found");
                }
                WiFi_StartTask("AT+CIPSEND", WIFI_TASK_WAIT_PROMPT,
                               WIFI_COMMAND_TIMEOUT_MS);
                return;
            }
            break;

        case WIFI_TASK_WAIT_PROMPT:
            if(WiFi_Contains(s_response, ">"))
            {
                s_transparent_mode = 1;
                s_task_state = WIFI_TASK_IDLE;
                s_retry_count = 0;
                s_payload_idle_ms = 0;
                WiFi_AtRxClear();
                Comm_Clear(COMM_SOURCE_WIFI);
                Comm_SendLine(COMM_SOURCE_SERIAL,
                              "wifi: connected to demo TCP server");
                return;
            }
            break;

        default:
            WiFi_FinishTask(0);
            return;
    }

    if(s_task_tick >= s_task_timeout)
    {
        if(s_task_state == WIFI_TASK_WAIT_JOIN)
            s_network_ready = 0;
        WiFi_FinishTask(0);
    }
}

uint8_t WiFi_HandleNotice(const char *line)
{
    if(strcmp(line, "ready") == 0 ||
       strcmp(line, "WIFI DISCONNECT") == 0)
    {
        WiFi_RecoverLink("wifi: module reset, full reconnect");
        return 1;
    }

    if(strcmp(line, "CLOSED") == 0 ||
       strcmp(line, "0,CLOSED") == 0 ||
       strcmp(line, "CONNECTION CLOSED") == 0 ||
       strstr(line, "CLOSED") != 0)
    {
        s_transparent_mode = 0;
        s_task_state = WIFI_TASK_IDLE;
        s_retry_tick = 0;
        s_retry_count = 0;
        s_payload_idle_ms = 0;
        Comm_Clear(COMM_SOURCE_WIFI);
        Comm_SendLine(COMM_SOURCE_SERIAL, "wifi: server disconnected");
        return 1;
    }

    return 0;
}

void WiFi_NotifyReceived(void)
{
    if(s_transparent_mode)
        s_payload_idle_ms = 0;
}

uint8_t WiFi_IsConnected(void)
{
    return s_transparent_mode;
}

uint8_t WiFi_IsServerReady(void)
{
    return s_network_ready;
}

WiFiState WiFi_GetState(void)
{
    if(s_suspended)
        return WIFI_STATE_DISABLED;
    if(s_transparent_mode)
        return WIFI_STATE_CONNECTED;
    if(s_network_ready)
        return WIFI_STATE_WAITING_TCP;
    if(s_module_ready)
        return WIFI_STATE_AP_NOT_JOINED;
    return WIFI_STATE_NO_MODULE;
}
