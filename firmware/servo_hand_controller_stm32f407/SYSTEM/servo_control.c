#include "servo_control.h"
#include "pca9685.h"

/* ===================== 位置舵机状态，仅0～4号 ===================== */
static uint8_t s_current_angle[APP_POSITION_CHANNEL_COUNT];
static uint8_t s_target_angle[APP_POSITION_CHANNEL_COUNT];

/* ===================== 5号连续舵机状态 ===================== */
static uint8_t s_speed_cmd = APP_SPEED_STOP_CMD;
static uint16_t s_stop_us = APP_SPEED_STOP_DEFAULT_US;
static uint16_t s_last_pulse_us = APP_SPEED_STOP_DEFAULT_US;
static uint8_t s_raw_pulse_mode;
static uint8_t s_output_disabled;

/*
 * 用户已经调好的位置参数，集中放在这里，便于检查和修改。
 * 2号通道保持禁用，5号连续舵机不使用以下角度参数。
 */
static const uint8_t s_servo_enabled[APP_SERVO_COUNT] =
{
    1,      /* 通道0 */
    1,      /* 通道1 */
    0,      /* 通道2：不转动，保持禁用 */
    1,      /* 通道3 */
    1,      /* 通道4 */
    1       /* 通道5：360°连续舵机 */
};

static const uint8_t s_home_angle[APP_SERVO_COUNT] =
{
    65,     /* 通道0 */
    25,     /* 通道1 */
    0,      /* 通道2禁用 */
    43,     /* 通道3 */
    40,     /* 通道4 */
    90      /* 原值保留，5号不按角度使用 */
};

static const uint8_t s_min_angle[APP_SERVO_COUNT] =
{
    45,     /* 通道0 */
    0,      /* 通道1 */
    0,      /* 通道2禁用 */
    0,      /* 通道3 */
    0,      /* 通道4 */
    90      /* 原值保留，5号不按角度使用 */
};

static const uint8_t s_max_angle[APP_SERVO_COUNT] =
{
    120,    /* 通道0 */
    45,     /* 通道1 */
    0,     /* 通道2禁用 */
    85,     /* 通道3 */
    80,     /* 通道4 */
    90      /* 原值保留，5号不按角度使用 */
};

/* ===================== 位置舵机演示状态 ===================== */
typedef struct
{
    uint8_t ch;
    uint8_t first;
    uint8_t second;
} DemoMove;

static const DemoMove s_demo_moves[] =
{
    {0, 120, 45},
    {1, 0,   45},
    {3, 0,   85},
    {4, 0,   80}
};

#define SERVO_DEMO_COUNT \
    ((uint8_t)(sizeof(s_demo_moves) / sizeof(s_demo_moves[0])))

static uint8_t s_demo_on;
static uint8_t s_demo_index;
static uint8_t s_demo_stage;
static uint16_t s_demo_tick;
static uint16_t s_demo_pause;
static uint8_t s_demo_ch5_stage;
static uint16_t s_demo_ch5_tick;
static uint8_t s_demo_repeat = 1;
static uint8_t s_ch5_once_running;
static uint8_t s_sensor_valid;
static uint8_t s_sensor_bend[APP_SENSOR_COUNT];

/* 传感器顺序与机械手手指顺序相反，S3对应禁用的CH2 */
static const uint8_t s_sensor_to_servo[APP_SENSOR_COUNT] =
{
    4, 3, 0xFF, 1, 0
};

/* 1表示该路弯曲量越大，目标角度越小 */
static const uint8_t s_sensor_angle_reverse[APP_SENSOR_COUNT] =
{
    0, 1, 0, 1, 0
};

/*
 * 5号MG996R连续舵机速度表，偏移量均相对停止脉宽1500us：
 * 速度命令0→1000us，20→1100us，...，80→1400us，90→1500us；
 * 100→1600us，...，160→1900us，180→2000us。
 * 命令越接近90，转速越慢；90为停止。
 */
typedef struct
{
    uint8_t speed_cmd;
    int16_t offset_us;
} SpeedPulsePoint;

static const SpeedPulsePoint s_speed_pulse_table[] =
{
    {0,   -500},
    {20,  -400},
    {40,  -300},
    {60,  -200},
    {80,  -100},
    {90,     0},
    {100,  100},
    {120,  200},
    {140,  300},
    {160,  400},
    {180,  500}
};

#define SPEED_PULSE_POINT_COUNT \
    ((uint8_t)(sizeof(s_speed_pulse_table) / sizeof(s_speed_pulse_table[0])))

static uint8_t Servo_ClampAngle(uint8_t ch, int angle)
{
    if(ch >= APP_POSITION_CHANNEL_COUNT)
        return 0;
    if(angle < s_min_angle[ch])
        angle = s_min_angle[ch];
    if(angle > s_max_angle[ch])
        angle = s_max_angle[ch];
    return (uint8_t)angle;
}

static uint8_t Servo_ClampSpeed(int speed)
{
    if(speed < 0)
        speed = 0;
    if(speed > 180)
        speed = 180;
    return (uint8_t)speed;
}

static uint16_t Servo_ContinuousPulse(uint8_t speed)
{
    uint8_t i;
    int offset = 0;
    int command_span;
    int delta_offset;
    int pulse;

    if(speed >= (APP_SPEED_STOP_CMD - APP_SPEED_DEADBAND_DEG) &&
       speed <= (APP_SPEED_STOP_CMD + APP_SPEED_DEADBAND_DEG))
    {
        offset = 0;
    }
    else
    {
        for(i = 0; i < (SPEED_PULSE_POINT_COUNT - 1); i++)
        {
            if(speed <= s_speed_pulse_table[i + 1].speed_cmd)
            {
                command_span =
                    (int)s_speed_pulse_table[i + 1].speed_cmd -
                    (int)s_speed_pulse_table[i].speed_cmd;
                delta_offset =
                    (int)s_speed_pulse_table[i + 1].offset_us -
                    (int)s_speed_pulse_table[i].offset_us;
                offset = (int)s_speed_pulse_table[i].offset_us +
                         ((int)speed -
                          (int)s_speed_pulse_table[i].speed_cmd) *
                         delta_offset / command_span;
                break;
            }
        }
    }

    pulse = (int)s_stop_us + offset;

    if(pulse < PCA9685_SERVO_MIN_US)
        pulse = PCA9685_SERVO_MIN_US;
    if(pulse > PCA9685_SERVO_MAX_US)
        pulse = PCA9685_SERVO_MAX_US;

    return (uint16_t)pulse;
}

static void Servo_ApplyContinuous(void)
{
    s_output_disabled = 0;
    s_last_pulse_us = Servo_ContinuousPulse(s_speed_cmd);
    PCA9685_SetPulseUs(APP_CONTINUOUS_CHANNEL, s_last_pulse_us);
}

static void Servo_StopContinuous(void)
{
    s_speed_cmd = APP_SPEED_STOP_CMD;
    s_raw_pulse_mode = 0;
    Servo_ApplyContinuous();
}

static void Servo_LockPositionTargets(void)
{
    uint8_t ch;

    for(ch = 0; ch < APP_POSITION_CHANNEL_COUNT; ch++)
    {
        if(s_servo_enabled[ch])
            s_target_angle[ch] = s_current_angle[ch];
    }
}

static void Servo_CancelDemo(void)
{
    if(!s_demo_on)
        return;

    s_demo_on = 0;
    s_demo_tick = 0;
    s_demo_ch5_stage = 0;
    s_demo_ch5_tick = 0;
    s_ch5_once_running = 0;
    Servo_LockPositionTargets();
    Servo_StopContinuous();
}

static uint8_t Servo_SetChannelValueInternal(uint8_t ch, int value);

static void Servo_BeginContinuousDemo(void)
{
    s_demo_ch5_stage = 1;
    s_demo_ch5_tick = 0;
    (void)Servo_SetChannelValueInternal(
        APP_CONTINUOUS_CHANNEL, APP_DEMO_CH5_SPEED_A);
}

static uint8_t Servo_SetChannelValueInternal(uint8_t ch, int value)
{
    if(ch >= APP_SERVO_COUNT || !s_servo_enabled[ch])
        return 0;

    if(ch == APP_CONTINUOUS_CHANNEL)
    {
        if(value >= PCA9685_SERVO_MIN_US &&
           value <= PCA9685_SERVO_MAX_US)
        {
            s_output_disabled = 0;
            s_raw_pulse_mode = 1;
            s_last_pulse_us = (uint16_t)value;
            PCA9685_SetPulseUs(APP_CONTINUOUS_CHANNEL, s_last_pulse_us);
            return 1;
        }

        if(value < 0 || value > 180)
            return 0;

        s_raw_pulse_mode = 0;
        s_speed_cmd = Servo_ClampSpeed(value);
        Servo_ApplyContinuous();
        return 1;
    }

    s_target_angle[ch] = Servo_ClampAngle(ch, value);
    return 1;
}

void Servo_ControlInit(void)
{
    uint8_t ch;

    s_demo_on = 0;
    s_demo_index = 0;
    s_demo_stage = 0;
    s_demo_tick = 0;
    s_demo_pause = 0;
    s_demo_ch5_stage = 0;
    s_demo_ch5_tick = 0;
    s_raw_pulse_mode = 0;
    s_output_disabled = 0;

    for(ch = 0; ch < APP_SERVO_COUNT; ch++)
    {
        if(!s_servo_enabled[ch])
            continue;

        if(ch == APP_CONTINUOUS_CHANNEL)
        {
            Servo_StopContinuous();
        }
        else
        {
            s_current_angle[ch] = Servo_ClampAngle(ch, s_home_angle[ch]);
            s_target_angle[ch] = s_current_angle[ch];
            PCA9685_SetAngle(ch, s_current_angle[ch]);
        }
    }
}

uint8_t Servo_SetChannelValue(uint8_t ch, int value)
{
    if(ch >= APP_SERVO_COUNT || !s_servo_enabled[ch])
        return 0;

    Servo_CancelDemo();
    return Servo_SetChannelValueInternal(ch, value);
}

uint8_t Servo_SetAllValue(int value)
{
    uint8_t ch;
    uint8_t count = 0;

    Servo_CancelDemo();
    for(ch = 0; ch < APP_SERVO_COUNT; ch++)
    {
        if(Servo_SetChannelValueInternal(ch, value))
            count++;
    }
    return count;
}

void Servo_UpdatePositions(void)
{
    uint8_t ch;
    int cur;
    int target;
    int diff;
    int step;
    int next;

    if(s_demo_on)
        return;

    for(ch = 0; ch < APP_POSITION_CHANNEL_COUNT; ch++)
    {
        if(!s_servo_enabled[ch])
            continue;

        cur = (int)s_current_angle[ch];
        target = (int)s_target_angle[ch];
        if(cur == target)
            continue;

        diff = target - cur;
        step = (diff > APP_POSITION_SLOW_ZONE ||
                diff < -APP_POSITION_SLOW_ZONE) ?
               APP_POSITION_FAST_STEP : APP_POSITION_SLOW_STEP;
        next = (diff > 0) ? (cur + step) : (cur - step);

        if((diff > 0 && next > target) || (diff < 0 && next < target))
            next = target;

        s_current_angle[ch] = (uint8_t)next;
        PCA9685_SetAngle(ch, s_current_angle[ch]);
    }
}

void Servo_StartDemo(void)
{
    s_ch5_once_running = 0;
    s_demo_repeat = 1;
    s_demo_on = 1;
    s_demo_index = 0;
    s_demo_stage = 0;
    s_demo_tick = 0;
    s_demo_pause = 0;
    s_demo_ch5_stage = 0;
    s_demo_ch5_tick = 0;

    Servo_LockPositionTargets();
    Servo_StopContinuous();
}

void Servo_StartDemoOnce(void)
{
    s_ch5_once_running = 0;
    s_demo_repeat = 0;
    s_demo_on = 1;
    s_demo_index = 0;
    s_demo_stage = 0;
    s_demo_tick = 0;
    s_demo_pause = 0;
    s_demo_ch5_stage = 0;
    s_demo_ch5_tick = 0;

    Servo_LockPositionTargets();
    Servo_StopContinuous();
}

void Servo_StartContinuousDemo(void)
{
    s_ch5_once_running = 0;
    s_demo_repeat = 1;
    s_demo_on = 1;
    s_demo_index = 0;
    s_demo_stage = 0;
    s_demo_tick = 0;
    s_demo_pause = 0;

    Servo_LockPositionTargets();
    Servo_BeginContinuousDemo();
}

void Servo_StartContinuousOnce(void)
{
    s_ch5_once_running = 1;
    s_demo_repeat = 0;
    s_demo_on = 1;
    s_demo_index = 0;
    s_demo_stage = 0;
    s_demo_tick = 0;
    s_demo_pause = 0;

    Servo_LockPositionTargets();
    Servo_BeginContinuousDemo();
}

void Servo_StopAll(void)
{
    s_ch5_once_running = 0;
    s_demo_on = 0;
    s_demo_tick = 0;
    s_demo_ch5_stage = 0;
    s_demo_ch5_tick = 0;
    Servo_LockPositionTargets();
    Servo_StopContinuous();
}

void Servo_GoHome(void)
{
    uint8_t ch;

    Servo_StopAll();
    for(ch = 0; ch < APP_POSITION_CHANNEL_COUNT; ch++)
    {
        if(s_servo_enabled[ch])
            s_target_angle[ch] =
                Servo_ClampAngle(ch, s_home_angle[ch]);
    }
}

void Servo_DisableContinuousOutput(void)
{
    s_ch5_once_running = 0;
    s_demo_on = 0;
    s_demo_ch5_stage = 0;
    s_demo_ch5_tick = 0;
    Servo_LockPositionTargets();
    s_output_disabled = 1;
    s_raw_pulse_mode = 0;
    s_speed_cmd = APP_SPEED_STOP_CMD;
    s_last_pulse_us = 0;
    PCA9685_SetChannelOff(APP_CONTINUOUS_CHANNEL);
}

static void Servo_ApplySensorBends(void)
{
    uint8_t sensor;
    uint8_t ch;
    uint8_t angle;

    for(sensor = 0; sensor < APP_SENSOR_COUNT; sensor++)
    {
        ch = s_sensor_to_servo[sensor];
        if(ch == 0xFF || !s_servo_enabled[ch])
            continue;

        if(s_sensor_angle_reverse[sensor])
        {
            angle = (uint8_t)(s_max_angle[ch] -
                    ((uint16_t)(s_max_angle[ch] - s_min_angle[ch]) *
                     s_sensor_bend[sensor] + 50U) / 100U);
        }
        else
        {
            angle = (uint8_t)(s_min_angle[ch] +
                    ((uint16_t)(s_max_angle[ch] - s_min_angle[ch]) *
                     s_sensor_bend[sensor] + 50U) / 100U);
        }
        (void)Servo_SetChannelValueInternal(ch, angle);
    }
}

uint8_t Servo_SetSensorBends(
    const uint8_t bends[APP_SENSOR_COUNT])
{
    uint8_t i;
    uint8_t all_bent = 1;

    if(bends == 0)
        return 0;

    for(i = 0; i < APP_SENSOR_COUNT; i++)
    {
        if(bends[i] > 100U)
            return 0;
        s_sensor_bend[i] = bends[i];
        if(bends[i] < APP_GLOVE_ALL_BENT_ON)
            all_bent = 0;
    }

    s_sensor_valid = 1;

    /*
     * 五路保持全弯时让一次性模拟动作完整执行；
     * 任意一路松开后立即退出演示，恢复实时单指控制。
     */
    if(s_demo_on)
    {
        /*
         * CH5_ONCE是独立的四秒舵机序列，不能被紧接着到达的
         * 普通GLOVE数据取消，否则动作只会运行不到100ms。
         */
        if(s_ch5_once_running)
        {
            Servo_ApplySensorBends();
            return 1;
        }

        if(all_bent)
            return 1;
        Servo_CancelDemo();
    }

    Servo_ApplySensorBends();
    return 1;
}

void Servo_GetSensorBends(uint8_t bends[APP_SENSOR_COUNT],
                          uint8_t *valid)
{
    uint8_t i;

    if(valid != 0)
        *valid = s_sensor_valid;

    if(bends == 0)
        return;

    for(i = 0; i < APP_SENSOR_COUNT; i++)
        bends[i] = s_sensor_bend[i];
}

uint8_t Servo_IsDemoRunning(void)
{
    return s_demo_on;
}

void Servo_UpdateDemo(void)
{
    const DemoMove *move;
    uint8_t ch;
    uint8_t target;
    uint16_t hold_ms;

    if(!s_demo_on)
        return;

    /* 4号位置舵机完成后，执行5号连续舵机演示 */
    if(s_demo_ch5_stage != 0)
    {
        s_demo_ch5_tick =
            (uint16_t)(s_demo_ch5_tick + APP_TICK_MS);
        if(s_demo_ch5_stage == 2)
        {
            hold_ms = s_ch5_once_running ?
                      APP_CH5_ONCE_RIGHT_MS :
                      APP_DEMO_CH5_SPEED_B_MS;
        }
        else
        {
            hold_ms = s_ch5_once_running ?
                      APP_CH5_ONCE_LEFT_MS :
                      APP_DEMO_CH5_SPEED_A_MS;
        }
        if(s_demo_ch5_tick < hold_ms)
            return;
        s_demo_ch5_tick = 0;

        if(s_demo_ch5_stage == 1)
        {
            (void)Servo_SetChannelValueInternal(
                APP_CONTINUOUS_CHANNEL, APP_DEMO_CH5_SPEED_B);
            s_demo_ch5_stage = 2;
        }
        else if(s_demo_ch5_stage == 2)
        {
            (void)Servo_SetChannelValueInternal(
                APP_CONTINUOUS_CHANNEL, APP_DEMO_CH5_SPEED_A);
            s_demo_ch5_stage = 3;
        }
        else
        {
            Servo_StopContinuous();
            if(s_demo_repeat)
            {
                s_demo_ch5_stage = 0;
                s_demo_index = 0;
                s_demo_stage = 0;
                s_demo_pause = 0;
            }
            else
            {
                s_demo_on = 0;
                s_ch5_once_running = 0;
                s_demo_ch5_stage = 0;
                s_demo_index = 0;
                s_demo_stage = 0;
                s_demo_pause = 0;
                if(s_sensor_valid)
                    Servo_ApplySensorBends();
            }
        }
        return;
    }

    s_demo_tick = (uint16_t)(s_demo_tick + APP_TICK_MS);
    if(s_demo_tick < APP_DEMO_INTERVAL_MS)
        return;
    s_demo_tick = 0;

    move = &s_demo_moves[s_demo_index];
    ch = move->ch;

    switch(s_demo_stage)
    {
        case 0:
            target = move->first;
            if(s_current_angle[ch] == target)
            {
                s_demo_stage = 1;
                return;
            }
            break;

        case 1:
            target = move->second;
            if(s_current_angle[ch] == target)
            {
                s_demo_stage = 2;
                s_demo_pause = 0;
                return;
            }
            break;

        case 2:
            s_demo_pause =
                (uint16_t)(s_demo_pause + APP_DEMO_INTERVAL_MS);
            if(s_demo_pause >= APP_DEMO_PAUSE_MS)
            {
                s_demo_stage = 3;
                s_demo_pause = 0;
            }
            return;

        case 3:
            target = move->first;
            if(s_current_angle[ch] == target)
            {
                s_demo_stage = 4;
                s_demo_pause = 0;
                return;
            }
            break;

        case 4:
            s_demo_pause =
                (uint16_t)(s_demo_pause + APP_DEMO_INTERVAL_MS);
            if(s_demo_pause >= APP_DEMO_PAUSE_MS)
            {
                s_demo_stage = 0;
                s_demo_pause = 0;
                s_demo_index++;
                if(s_demo_index >= SERVO_DEMO_COUNT)
                {
                    /*
                     * 4号手指完成后依次执行：
                     * 100速度1秒 → 80速度2秒 → 100速度1秒 → 停止，再循环。
                     */
                    s_demo_index = 0;
                    Servo_BeginContinuousDemo();
                }
            }
            return;

        default:
            Servo_StopAll();
            return;
    }

    target = Servo_ClampAngle(ch, target);
    if(s_current_angle[ch] < target)
        s_current_angle[ch]++;
    else if(s_current_angle[ch] > target)
        s_current_angle[ch]--;

    PCA9685_SetAngle(ch, s_current_angle[ch]);
}

uint8_t Servo_SetContinuousTrim(int trim_us)
{
    if(trim_us < APP_TRIM_MIN_US || trim_us > APP_TRIM_MAX_US)
        return 0;

    Servo_CancelDemo();
    s_stop_us = (uint16_t)trim_us;
    s_raw_pulse_mode = 0;
    Servo_ApplyContinuous();
    return 1;
}

uint8_t Servo_GetPositionStatus(uint8_t ch, ServoPositionStatus *status)
{
    if(ch >= APP_POSITION_CHANNEL_COUNT || status == 0)
        return 0;

    status->enabled = s_servo_enabled[ch];
    status->current_angle = s_current_angle[ch];
    status->target_angle = s_target_angle[ch];
    status->min_angle = s_min_angle[ch];
    status->max_angle = s_max_angle[ch];
    return 1;
}

void Servo_GetContinuousStatus(ServoContinuousStatus *status)
{
    if(status == 0)
        return;

    status->speed_cmd = s_speed_cmd;
    status->pulse_us = s_last_pulse_us;
    status->stop_us = s_stop_us;
    status->raw_pulse_mode = s_raw_pulse_mode;
    status->output_disabled = s_output_disabled;
}
