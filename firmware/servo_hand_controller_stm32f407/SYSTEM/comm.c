#include "comm.h"
#include "usart.h"

/* 每路接收环形缓冲区，中断只负责收字节，主循环负责解析完整命令 */
#define COMM_RX_BUFFER_SIZE  512
#define COMM_LINE_BUFFER_SIZE 128
#define COMM_IDLE_END_TICKS  5

static uint8_t s_rx_buffer[COMM_SOURCE_COUNT][COMM_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head[COMM_SOURCE_COUNT];
static volatile uint16_t s_rx_tail[COMM_SOURCE_COUNT];
static volatile uint32_t s_rx_byte_count[COMM_SOURCE_COUNT];
static char s_line_buffer[COMM_SOURCE_COUNT][COMM_LINE_BUFFER_SIZE];
static uint16_t s_line_length[COMM_SOURCE_COUNT];
static uint8_t s_line_ready[COMM_SOURCE_COUNT];
static uint8_t s_idle_ticks[COMM_SOURCE_COUNT];
static uint8_t s_discard_line[COMM_SOURCE_COUNT];
static uint32_t s_bluetooth_baud;

void Comm_Init(void)
{
    uint8_t i;

    for(i = 0; i < COMM_SOURCE_COUNT; i++)
    {
        s_rx_head[i] = 0;
        s_rx_tail[i] = 0;
        s_rx_byte_count[i] = 0;
        s_line_length[i] = 0;
        s_line_ready[i] = 0;
        s_idle_ticks[i] = 0;
        s_discard_line[i] = 0;
    }
}

void Comm_InitBluetooth(uint32_t baud)
{
    init_usart2(baud);
    s_bluetooth_baud = baud;
    Comm_Clear(COMM_SOURCE_BLUETOOTH);
}

uint32_t Comm_GetBluetoothBaud(void)
{
    return s_bluetooth_baud;
}

void Comm_RxByteFromISR(CommSource source, uint8_t byte)
{
    uint16_t head;
    uint16_t next;

    if(source >= COMM_SOURCE_COUNT)
    {
        return;
    }

    head = s_rx_head[source];
    next = (uint16_t)((head + 1) % COMM_RX_BUFFER_SIZE);
    if(next != s_rx_tail[source])
    {
        s_rx_buffer[source][head] = byte;
        s_rx_head[source] = next;
        s_rx_byte_count[source]++;
    }
}

uint32_t Comm_GetRxByteCount(CommSource source)
{
    if(source >= COMM_SOURCE_COUNT)
        return 0;
    return s_rx_byte_count[source];
}

uint8_t Comm_ReadLine(CommSource source, char *line, uint16_t line_size)
{
    uint16_t head;
    uint16_t tail;
    uint16_t copy_length;
    uint8_t byte;
    uint8_t received_byte = 0;

    if(source >= COMM_SOURCE_COUNT || line_size == 0)
    {
        return 0;
    }

    if(s_line_ready[source])
    {
        copy_length = s_line_length[source];
        if(copy_length >= line_size)
            copy_length = (uint16_t)(line_size - 1);

        for(head = 0; head < copy_length; head++)
            line[head] = s_line_buffer[source][head];
        line[copy_length] = '\0';

        s_line_length[source] = 0;
        s_line_ready[source] = 0;
        s_idle_ticks[source] = 0;
        return 1;
    }

    head = s_rx_head[source];
    tail = s_rx_tail[source];

    while(tail != head)
    {
        byte = s_rx_buffer[source][tail];
        tail = (uint16_t)((tail + 1) % COMM_RX_BUFFER_SIZE);
        s_rx_tail[source] = tail;
        received_byte = 1;
        s_idle_ticks[source] = 0;

        if(byte == '\r' || byte == '\n')
        {
            if(s_discard_line[source])
            {
                s_discard_line[source] = 0;
                s_line_length[source] = 0;
                continue;
            }

            if(s_line_length[source] > 0)
            {
                s_line_buffer[source][s_line_length[source]] = '\0';
                s_line_ready[source] = 1;
                break;
            }
            continue;
        }

        if(s_discard_line[source])
        {
            continue;
        }

        if(s_line_length[source] < (COMM_LINE_BUFFER_SIZE - 1))
        {
            s_line_buffer[source][s_line_length[source]++] = (char)byte;

            /*
             * 手机蓝牙串口不一定发送回车或换行。
             * PING是链路诊断命令，收到完整四个字节后立即应答。
             */
            if(source == COMM_SOURCE_BLUETOOTH &&
               s_line_length[source] == 4 &&
               (s_line_buffer[source][0] == 'P' ||
                s_line_buffer[source][0] == 'p') &&
               (s_line_buffer[source][1] == 'I' ||
                s_line_buffer[source][1] == 'i') &&
               (s_line_buffer[source][2] == 'N' ||
                s_line_buffer[source][2] == 'n') &&
               (s_line_buffer[source][3] == 'G' ||
                s_line_buffer[source][3] == 'g'))
            {
                s_line_buffer[source][s_line_length[source]] = '\0';
                s_line_ready[source] = 1;
                break;
            }
        }
        else
        {
            /* 超长命令整行丢弃，避免残片被错误执行 */
            s_discard_line[source] = 1;
            s_line_length[source] = 0;
        }
    }

    /*
     * 串口助手不一定发送回车或换行。
     * 手机蓝牙可能把一条命令拆成多个BLE通知。连续约50 ms没有
     * 新字节时才结束并解析，避免"PING"被拆成"P"和"ING"。
     */
    if(!received_byte && s_line_length[source] > 0)
    {
        s_idle_ticks[source]++;
        if(s_idle_ticks[source] >= COMM_IDLE_END_TICKS)
        {
            s_line_buffer[source][s_line_length[source]] = '\0';
            s_line_ready[source] = 1;
        }
    }

    if(s_line_ready[source])
    {
        copy_length = s_line_length[source];
        if(copy_length >= line_size)
            copy_length = (uint16_t)(line_size - 1);

        for(head = 0; head < copy_length; head++)
            line[head] = s_line_buffer[source][head];
        line[copy_length] = '\0';

        s_line_length[source] = 0;
        s_line_ready[source] = 0;
        s_idle_ticks[source] = 0;
        return 1;
    }

    return 0;
}

void Comm_Send(CommSource source, const char *text)
{
    if(text == 0)
    {
        return;
    }

    switch(source)
    {
        case COMM_SOURCE_SERIAL:
            send_str((char *)text);
            break;

        case COMM_SOURCE_BLUETOOTH:
            send2_str((char *)text);
            break;

        case COMM_SOURCE_WIFI:
            send3_str((char *)text);
            break;

        default:
            break;
    }
}

void Comm_SendLine(CommSource source, const char *text)
{
    Comm_Send(source, text);
    Comm_Send(source, "\r\n");
}

void Comm_Clear(CommSource source)
{
    if(source >= COMM_SOURCE_COUNT)
    {
        return;
    }

    s_rx_tail[source] = s_rx_head[source];
    s_line_length[source] = 0;
    s_line_ready[source] = 0;
    s_idle_ticks[source] = 0;
    s_discard_line[source] = 0;
}

const char *Comm_SourceName(CommSource source)
{
    switch(source)
    {
        case COMM_SOURCE_SERIAL:
            return "serial";

        case COMM_SOURCE_BLUETOOTH:
            return "bluetooth";

        case COMM_SOURCE_WIFI:
            return "wifi";

        default:
            return "unknown";
    }
}
