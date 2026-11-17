#include "command.h"
#include "servo_control.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define COMMAND_CAL_FINE_START_US    1670
#define COMMAND_CAL_FINE_END_US      1810
#define COMMAND_CAL_FINE_STEP_US     5
#define COMMAND_CAL_COARSE_START_US  1000
#define COMMAND_CAL_COARSE_END_US    2000
#define COMMAND_CAL_COARSE_STEP_US   10
#define COMMAND_CAL_INTERVAL_MS      1000

static uint8_t s_calibration_on;
static uint16_t s_calibration_pulse_us;
static uint16_t s_calibration_end_us;
static uint16_t s_calibration_step_us;
static uint16_t s_calibration_tick;
static CommSource s_calibration_source;
static uint32_t s_glove_rx_count;
static CommSource s_last_glove_source;
static uint8_t s_glove_source_valid;
static uint8_t s_glove_control_enabled = 1;

static char *Command_TrimText(char *text)
{
    char *end;

    while(*text == ' ' || *text == '\t')
        text++;

    end = text + strlen(text);
    while(end > text && (end[-1] == ' ' || end[-1] == '\t'))
        end--;
    *end = '\0';
    return text;
}

static void Command_ToUpper(char *text)
{
    while(*text != '\0')
    {
        if(*text >= 'a' && *text <= 'z')
            *text = (char)(*text - 'a' + 'A');
        text++;
    }
}

static void Command_StripWifiEnvelope(char *text)
{
    char *payload;

    if(strncmp(text, "+IPD,", 5) != 0)
        return;

    payload = strchr(text, ':');
    if(payload == NULL)
        return;

    memmove(text, payload + 1, strlen(payload));
}

static uint8_t Command_ParseBatch(char *text)
{
    int ch;
    int value;
    uint8_t count = 0;
    char *token;

    token = strtok(text, ",");
    while(token != NULL)
    {
        if(sscanf(token, "%d:%d", &ch, &value) == 2 ||
           sscanf(token, "%d=%d", &ch, &value) == 2)
        {
            if(ch >= 0 && ch < APP_SERVO_COUNT)
            {
                if(Servo_SetChannelValue((uint8_t)ch, value))
                    count++;
            }
        }
        token = strtok(NULL, ",");
    }
    return count;
}

static void Command_StartCalibration(CommSource source,
                                     uint16_t start_us,
                                     uint16_t end_us,
                                     uint16_t step_us,
                                     const char *name)
{
    char line[72];

    s_calibration_on = 1;
    s_calibration_pulse_us = start_us;
    s_calibration_end_us = end_us;
    s_calibration_step_us = step_us;
    s_calibration_tick = 0;
    s_calibration_source = source;
    (void)Servo_SetChannelValue(APP_CONTINUOUS_CHANNEL,
                                s_calibration_pulse_us);

    snprintf(line, sizeof(line), "CH5 %s start, pulse=%uus",
             name, (unsigned int)s_calibration_pulse_us);
    Comm_SendLine(source, line);
}

static void Command_SendStatus(CommSource source)
{
    uint8_t ch;
    char line[128];
    ServoPositionStatus position;
    ServoContinuousStatus continuous;
    uint8_t sensor_bends[APP_SENSOR_COUNT];
    uint8_t sensor_valid;

    Comm_SendLine(source, "servo status:");

    for(ch = 0; ch < APP_POSITION_CHANNEL_COUNT; ch++)
    {
        if(!Servo_GetPositionStatus(ch, &position))
            continue;

        if(!position.enabled)
        {
            snprintf(line, sizeof(line), "CH%u disabled", (unsigned int)ch);
        }
        else
        {
            snprintf(line, sizeof(line),
                     "CH%u angle cur=%u target=%u range=%u-%u",
                     (unsigned int)ch,
                     (unsigned int)position.current_angle,
                     (unsigned int)position.target_angle,
                     (unsigned int)position.min_angle,
                     (unsigned int)position.max_angle);
        }
        Comm_SendLine(source, line);
    }

    Servo_GetContinuousStatus(&continuous);
    if(continuous.output_disabled)
    {
        snprintf(line, sizeof(line), "CH%u output disabled",
                 (unsigned int)APP_CONTINUOUS_CHANNEL);
    }
    else if(continuous.raw_pulse_mode)
    {
        snprintf(line, sizeof(line),
                 "CH%u raw pulse=%uus trim=%uus",
                 (unsigned int)APP_CONTINUOUS_CHANNEL,
                 (unsigned int)continuous.pulse_us,
                 (unsigned int)continuous.stop_us);
    }
    else
    {
        snprintf(line, sizeof(line),
                 "CH%u speed=%u pulse=%uus trim=%uus (90=stop)",
                 (unsigned int)APP_CONTINUOUS_CHANNEL,
                 (unsigned int)continuous.speed_cmd,
                 (unsigned int)continuous.pulse_us,
                 (unsigned int)continuous.stop_us);
    }
    Comm_SendLine(source, line);

    Servo_GetSensorBends(sensor_bends, &sensor_valid);
    if(sensor_valid)
    {
        snprintf(line, sizeof(line),
                 "GLOVE last: S1=%u S2=%u S3=%u S4=%u S5=%u",
                 (unsigned int)sensor_bends[0],
                 (unsigned int)sensor_bends[1],
                 (unsigned int)sensor_bends[2],
                 (unsigned int)sensor_bends[3],
                 (unsigned int)sensor_bends[4]);
    }
    else
    {
        snprintf(line, sizeof(line), "GLOVE last: no data");
    }
    Comm_SendLine(source, line);

    if(s_glove_source_valid)
    {
        snprintf(line, sizeof(line),
                 "GLOVE rx count=%lu source=%s control=%s demo=%s",
                 (unsigned long)s_glove_rx_count,
                 Comm_SourceName(s_last_glove_source),
                 s_glove_control_enabled ? "on" : "off",
                 Servo_IsDemoRunning() ? "run" : "stop");
    }
    else
    {
        if(s_glove_control_enabled)
            snprintf(line, sizeof(line), "GLOVE rx count=0");
        else
            snprintf(line, sizeof(line),
                     "GLOVE control=off, send WIFION to restore");
    }
    Comm_SendLine(source, line);
}

static void Command_HandleLine(CommSource source, char *text)
{
    int ch;
    int baud;
    int value;
    int trim_us;
    int parsed;
    uint8_t count;
    uint8_t bends[APP_SENSOR_COUNT];
    uint8_t at_ok;
    char *command;
    const char *at_command;
    char upper[APP_COMMAND_SIZE];
    char at_response[APP_WIFI_AT_RESPONSE_SIZE];
    ServoPositionStatus position;
    ServoContinuousStatus continuous;
    WiFiState wifi_state;

    command = Command_TrimText(text);
    if(*command == '\0')
        return;

    /*
     * 手套USB日志与有线控制共用USART1文本链路。
     * 所有方括号诊断行必须忽略，避免被当成无效命令反复回复。
     */
    if(command[0] == '[' || command[0] == '=')
        return;

    if(source == COMM_SOURCE_WIFI)
        Command_StripWifiEnvelope(command);

    strncpy(upper, command, APP_COMMAND_SIZE - 1);
    upper[APP_COMMAND_SIZE - 1] = '\0';
    Command_ToUpper(upper);

    if(strcmp(upper, "PING") == 0)
    {
        snprintf(upper, sizeof(upper), "PONG %s", APP_VERSION);
        Comm_SendLine(source, upper);
        return;
    }

    if(strcmp(upper, "CAL5") == 0)
    {
        Command_StartCalibration(source,
                                 COMMAND_CAL_COARSE_START_US,
                                 COMMAND_CAL_COARSE_END_US,
                                 COMMAND_CAL_COARSE_STEP_US,
                                 "coarse calibration");
        return;
    }

    if(strcmp(upper, "FINE5") == 0)
    {
        Command_StartCalibration(source,
                                 COMMAND_CAL_FINE_START_US,
                                 COMMAND_CAL_FINE_END_US,
                                 COMMAND_CAL_FINE_STEP_US,
                                 "fine calibration");
        return;
    }

    if(strcmp(upper, "OFF5") == 0)
    {
        s_calibration_on = 0;
        Servo_DisableContinuousOutput();
        Comm_SendLine(source, "CH5 output disabled, 90 still re-enables");
        return;
    }

    if(strcmp(upper, "STATUS") == 0)
    {
        Command_SendStatus(source);
        return;
    }

    if(strcmp(upper, "WIFI?") == 0)
    {
        wifi_state = WiFi_GetState();
        if(wifi_state == WIFI_STATE_CONNECTED)
            snprintf(upper, sizeof(upper),
                     "wifi: TCP connected, uart=%u",
                     (unsigned int)WiFi_GetBaud());
        else if(wifi_state == WIFI_STATE_WAITING_TCP)
            snprintf(upper, sizeof(upper),
                     "wifi: AP joined, waiting TCP, uart=%u",
                     (unsigned int)WiFi_GetBaud());
        else if(wifi_state == WIFI_STATE_AP_NOT_JOINED)
            snprintf(upper, sizeof(upper),
                     "wifi: module OK, AP join failed, uart=%u",
                     (unsigned int)WiFi_GetBaud());
        else if(wifi_state == WIFI_STATE_DISABLED)
            snprintf(upper, sizeof(upper),
                     "wifi: disabled by WIFIOFF, uart=%u",
                     (unsigned int)WiFi_GetBaud());
        else
            snprintf(upper, sizeof(upper),
                     "wifi: module no response, uart=%u",
                     (unsigned int)WiFi_GetBaud());
        Comm_SendLine(source, upper);
        return;
    }

    if(strcmp(upper, "WIFIFIX") == 0)
    {
        /*
         * 不改变当前波特率，重新执行透传退出、旧连接关闭和入网流程。
         * 适合运行中WiFi链路卡住时从蓝牙直接恢复。
         */
        (void)WiFi_SetBaud(WiFi_GetBaud());
        Comm_SendLine(source,
                      "wifi recovery started: exit transparent, reconnect");
        return;
    }

    if(strcmp(upper, "WIFIOFF") == 0)
    {
        s_glove_control_enabled = 0;
        Servo_StopAll();
        WiFi_Suspend();
        Comm_SendLine(source,
                      "wifi disabled, all GLOVE data ignored until WIFION");
        return;
    }

    if(strcmp(upper, "WIFION") == 0)
    {
        s_glove_control_enabled = 1;
        WiFi_Resume();
        Comm_SendLine(source,
                      "wifi enabled, GLOVE control restored, reconnect started");
        return;
    }

    if(strncmp(upper, "WIFIBAUD:", 9) == 0)
    {
        if(sscanf(command + 9, "%d", &baud) != 1 ||
           (baud != 4800 && baud != 9600 && baud != 19200 &&
            baud != 38400 && baud != 57600 && baud != 74880 &&
            baud != 115200 && baud != 230400))
        {
            Comm_SendLine(source,
                          "invalid WiFi baud; use 9600/38400/74880/115200");
            return;
        }

        Comm_SendLine(source, "changing WiFi UART baud...");
        snprintf(upper, sizeof(upper), "wifi init result=%u, uart=%u",
                 (unsigned int)WiFi_SetBaud((uint32_t)baud),
                 (unsigned int)WiFi_GetBaud());
        Comm_SendLine(source, upper);
        return;
    }

    if(strcmp(upper, "WIFIAT") == 0 ||
       strncmp(upper, "WIFIAT:", 7) == 0)
    {
        if(source == COMM_SOURCE_WIFI)
        {
            Comm_SendLine(COMM_SOURCE_SERIAL,
                          "send WIFIAT from Bluetooth or USB serial");
            return;
        }

        at_command = (strcmp(upper, "WIFIAT") == 0) ?
                     "AT" : (command + 7);
        if(*at_command == '\0')
        {
            Comm_SendLine(source, "missing AT command");
            return;
        }

        Comm_SendLine(source, "sending AT to WiFi module...");
        at_ok = WiFi_ATCommand(at_command, at_response,
                               sizeof(at_response), 3000);
        if(at_ok)
        {
            Comm_Send(source, "[WIFI AT RX]\r\n");
            Comm_Send(source, at_response);
            Comm_Send(source, "\r\n");
        }
        else if(at_response[0] != '\0')
        {
            Comm_Send(source, "[WIFI AT RX partial]\r\n");
            Comm_Send(source, at_response);
            Comm_Send(source, "\r\nWiFi AT incomplete response\r\n");
        }
        else
        {
            Comm_SendLine(source, "WiFi AT no response");
        }
        return;
    }

    if(strcmp(upper, "BT?") == 0)
    {
        snprintf(upper, sizeof(upper), "bluetooth baud=%u",
                 (unsigned int)Comm_GetBluetoothBaud());
        Comm_SendLine(source, upper);
        return;
    }

    if(strcmp(upper, "BTTEST") == 0)
    {
        Comm_SendLine(COMM_SOURCE_BLUETOOTH,
                      "bluetooth output test OK");
        if(source != COMM_SOURCE_BLUETOOTH)
            Comm_SendLine(source, "bluetooth test sent");
        return;
    }

    if(strncmp(upper, "BTBAUD:", 7) == 0)
    {
        if(sscanf(command + 7, "%d", &baud) != 1 ||
           (baud != 4800 && baud != 9600 && baud != 19200 &&
            baud != 38400 && baud != 57600 && baud != 115200 &&
            baud != 230400))
        {
            Comm_SendLine(source,
                          "invalid BT baud; use 9600/19200/38400/57600/115200");
            return;
        }

        Comm_InitBluetooth((uint32_t)baud);
        snprintf(upper, sizeof(upper),
                 "bluetooth baud changed to %d, reconnect at this baud", baud);
        Comm_SendLine(COMM_SOURCE_SERIAL, upper);
        if(source != COMM_SOURCE_SERIAL)
            Comm_SendLine(source, upper);
        return;
    }

    if(strcmp(upper, "STOP") == 0)
    {
        Servo_StopAll();
        Comm_SendLine(source, "positions held, CH5 stopped");
        return;
    }

    if(strcmp(upper, "HOME") == 0)
    {
        Servo_GoHome();
        Comm_SendLine(source, "position servos home, CH5 stopped");
        return;
    }

    if(strcmp(upper, "D") == 0)
    {
        Servo_StartDemo();
        Comm_SendLine(source, "position demo start, CH5 stopped");
        return;
    }

    if(strcmp(upper, "DEMO_ONCE") == 0)
    {
        Servo_StartDemoOnce();
        Comm_SendLine(source, "one-shot demo start");
        return;
    }

    if(strncmp(upper, "GLOVE,", 6) == 0)
    {
        int b1, b2, b3, b4, b5;

        /*
         * WIFIOFF用于切换到纯蓝牙/命令控制。
         * 手套WiFi断开后会自动走USART1有线GLOVE，因此这里必须
         * 同时屏蔽所有来源的GLOVE，直到WIFION明确恢复。
         */
        if(!s_glove_control_enabled)
            return;

        if(sscanf(command + 6, "%d,%d,%d,%d,%d",
                  &b1, &b2, &b3, &b4, &b5) != APP_SENSOR_COUNT ||
           b1 < 0 || b1 > 100 ||
           b2 < 0 || b2 > 100 ||
           b3 < 0 || b3 > 100 ||
           b4 < 0 || b4 > 100 ||
           b5 < 0 || b5 > 100)
        {
            Comm_SendLine(source, "invalid glove data");
            return;
        }

        bends[0] = (uint8_t)b1;
        bends[1] = (uint8_t)b2;
        bends[2] = (uint8_t)b3;
        bends[3] = (uint8_t)b4;
        bends[4] = (uint8_t)b5;

        if(!Servo_SetSensorBends(bends))
        {
            Comm_SendLine(source, "glove apply failed");
            return;
        }

        s_glove_rx_count++;
        s_last_glove_source = source;
        s_glove_source_valid = 1;
        if(source == COMM_SOURCE_WIFI)
            WiFi_NotifyReceived();

        if(source != COMM_SOURCE_WIFI)
            Comm_SendLine(source, "glove values applied");
        return;
    }

    if(strcmp(upper, "DEMO5") == 0)
    {
        Servo_StartContinuousDemo();
        Comm_SendLine(source,
                      "CH5 demo: speed100 1s -> speed80 2s -> speed100 1s -> stop");
        return;
    }

    if(strcmp(upper, "CH5_ONCE") == 0)
    {
        Servo_StartContinuousOnce();
        Comm_SendLine(source,
                      "CH5 once: left 300ms -> right 600ms -> left 300ms -> stop");
        return;
    }

    if(strcmp(upper, "X") == 0)
    {
        Servo_StopAll();
        Comm_SendLine(source, "demo stop, positions held, CH5 stopped");
        return;
    }

    if(strncmp(upper, "TRIM:", 5) == 0)
    {
        if(sscanf(command + 5, "%d,%d", &ch, &trim_us) != 2 ||
           ch != APP_CONTINUOUS_CHANNEL ||
           !Servo_SetContinuousTrim(trim_us))
        {
            Comm_SendLine(source, "only CH5 supports TRIM, use TRIM:5,1500");
            return;
        }

        snprintf(upper, sizeof(upper), "CH%d trim = %dus",
                 APP_CONTINUOUS_CHANNEL, trim_us);
        Comm_SendLine(source, upper);
        return;
    }

    if(strncmp(upper, "ALL:", 4) == 0)
    {
        value = atoi(command + 4);
        count = Servo_SetAllValue(value);
        snprintf(upper, sizeof(upper), "all set %u channel(s)",
                 (unsigned int)count);
        Comm_SendLine(source, upper);
        return;
    }

    if(strchr(command, ':') != NULL || strchr(command, ',') != NULL)
    {
        count = Command_ParseBatch(command);
        if(count > 0)
        {
            snprintf(upper, sizeof(upper), "batch set %u channel(s)",
                     (unsigned int)count);
            Comm_SendLine(source, upper);
        }
        else
        {
            Comm_SendLine(source, "invalid batch command");
        }
        return;
    }

    parsed = sscanf(command, "%d %d", &ch, &value);
    if(parsed == 1)
    {
        value = ch;
        ch = 0;
    }
    else if(parsed != 2)
    {
        Comm_SendLine(source, "invalid command");
        return;
    }

    if(ch < 0 || ch >= APP_SERVO_COUNT)
    {
        Comm_SendLine(source, "channel invalid");
        return;
    }

    if(!Servo_SetChannelValue((uint8_t)ch, value))
    {
        Comm_SendLine(source, "channel disabled or value invalid");
        return;
    }

    if(ch == APP_CONTINUOUS_CHANNEL)
    {
        Servo_GetContinuousStatus(&continuous);
        if(continuous.raw_pulse_mode)
        {
            snprintf(upper, sizeof(upper), "CH%d pulse = %uus",
                     APP_CONTINUOUS_CHANNEL,
                     (unsigned int)continuous.pulse_us);
        }
        else
        {
            snprintf(upper, sizeof(upper), "CH%d speed = %u (90=stop)",
                     APP_CONTINUOUS_CHANNEL,
                     (unsigned int)continuous.speed_cmd);
        }
    }
    else
    {
        (void)Servo_GetPositionStatus((uint8_t)ch, &position);
        snprintf(upper, sizeof(upper), "CH%d target = %u deg",
                 ch, (unsigned int)position.target_angle);
    }
    Comm_SendLine(source, upper);
}

void Command_ProcessInput(CommSource source)
{
    char line[APP_COMMAND_SIZE];
    char check[APP_COMMAND_SIZE];
    char *trimmed;

    while(Comm_ReadLine(source, line, sizeof(line)))
    {
        if(source == COMM_SOURCE_WIFI && WiFi_HandleNotice(line))
            continue;

        if(s_calibration_on)
        {
            strncpy(check, line, APP_COMMAND_SIZE - 1);
            check[APP_COMMAND_SIZE - 1] = '\0';
            trimmed = Command_TrimText(check);
            Command_ToUpper(trimmed);
            if(strcmp(trimmed, "STATUS") != 0 &&
               strcmp(trimmed, "CAL5") != 0 &&
               strcmp(trimmed, "FINE5") != 0)
            {
                s_calibration_on = 0;
                Comm_SendLine(source, "CH5 calibration stopped");
            }
        }

        Command_HandleLine(source, line);
    }
}

void Command_UpdateCalibration(uint16_t elapsed_ms)
{
    if(!s_calibration_on)
        return;

    s_calibration_tick =
        (uint16_t)(s_calibration_tick + elapsed_ms);
    if(s_calibration_tick < COMMAND_CAL_INTERVAL_MS)
        return;
    s_calibration_tick = 0;

    if(s_calibration_pulse_us >= s_calibration_end_us)
    {
        s_calibration_on = 0;
        Servo_StopAll();
        Comm_SendLine(s_calibration_source,
                      "CH5 calibration finished, restored trim stop");
        return;
    }

    s_calibration_pulse_us =
        (uint16_t)(s_calibration_pulse_us + s_calibration_step_us);
    if(s_calibration_pulse_us > s_calibration_end_us)
        s_calibration_pulse_us = s_calibration_end_us;

    (void)Servo_SetChannelValue(APP_CONTINUOUS_CHANNEL,
                                s_calibration_pulse_us);
    {
        char line[48];
        snprintf(line, sizeof(line), "CH5 pulse=%uus",
                 (unsigned int)s_calibration_pulse_us);
        Comm_SendLine(s_calibration_source, line);
    }
}
