#include "pca9685.h"
#include "delay.h"

/* ===== 软件I2C引脚定义，需要换脚只改这里 ===== */
#define SCL_PIN    GPIO_Pin_8
#define SDA_PIN    GPIO_Pin_9
#define I2C_PORT   GPIOB

#define PCA9685_ADDR  0x40        // 7位地址，地址焊盘全悬空时

/* ---------- 引脚操作宏 ---------- */
#define SCL_H()  GPIO_SetBits(I2C_PORT, SCL_PIN)
#define SCL_L()  GPIO_ResetBits(I2C_PORT, SCL_PIN)
#define SDA_H()  GPIO_SetBits(I2C_PORT, SDA_PIN)
#define SDA_L()  GPIO_ResetBits(I2C_PORT, SDA_PIN)
#define SDA_READ()  GPIO_ReadInputDataBit(I2C_PORT, SDA_PIN)

/* ---------- 微秒级短延时，配合你工程的delay函数微调 ---------- */
static void i2c_delay(void)
{
    uint8_t t = 20;
    while(t--);
}

/* ---------- I2C GPIO初始化：开漏输出，板载已有上拉 ---------- */
static void I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // 开漏
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);

    SCL_H();
    SDA_H();
}

/* ---------- I2C 基本时序 ---------- */
static void I2C_Start(void)
{
    SDA_H(); SCL_H(); i2c_delay();
    SDA_L(); i2c_delay();
    SCL_L(); i2c_delay();
}

static void I2C_Stop(void)
{
    SDA_L(); SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();
}

static void I2C_SendByte(uint8_t dat)
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        if(dat & 0x80) SDA_H(); else SDA_L();
        dat <<= 1;
        i2c_delay();
        SCL_H(); i2c_delay();
        SCL_L(); i2c_delay();
    }
}

static uint8_t I2C_WaitAck(void)
{
    uint8_t ack;
    SDA_H();               // 释放SDA，让从机拉低
    i2c_delay();
    SCL_H(); i2c_delay();
    ack = SDA_READ();      // 0=应答成功
    SCL_L(); i2c_delay();
    return ack;
}

/* ---------- PCA9685 寄存器写 ---------- */
static void PCA9685_WriteReg(uint8_t reg, uint8_t val)
{
    I2C_Start();
    I2C_SendByte(PCA9685_ADDR << 1);
    I2C_WaitAck();
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(val);
    I2C_WaitAck();
    I2C_Stop();
}

/* ---------- 设置某通道PWM：on通常给0 ---------- */
void PCA9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off)
{
    if(ch > 15)
    {
        return;
    }

    I2C_Start();
    I2C_SendByte(PCA9685_ADDR << 1);
    I2C_WaitAck();
    I2C_SendByte(0x06 + 4*ch);   // LEDx_ON_L 寄存器，每通道占4字节
    I2C_WaitAck();
    I2C_SendByte(on & 0xFF);  I2C_WaitAck();
    I2C_SendByte(on >> 8);    I2C_WaitAck();
    I2C_SendByte(off & 0xFF); I2C_WaitAck();
    I2C_SendByte(off >> 8);   I2C_WaitAck();
    I2C_Stop();
}

/* ---------- 初始化：50Hz（20ms）舵机频率 ---------- */
void PCA9685_Init(void)
{
    I2C_GPIO_Init();
    delay_ms(10);

    PCA9685_WriteReg(0x00, 0x10);   // MODE1：进入睡眠
    PCA9685_WriteReg(0xFE, 122);    // 保持原工程参数，避免改变已调好的位置角度
    PCA9685_WriteReg(0x00, 0x00);    // 唤醒
    delay_ms(5);
    PCA9685_WriteReg(0x00, 0x20);  // 开启地址自增AI
    PCA9685_WriteReg(0x01, 0x04);   // MODE2：推挽输出
}

/* ---------- 原始脉宽设置，便于校准不同批次舵机 ---------- */
void PCA9685_SetPulseUs(uint8_t ch, uint16_t pulse_us)
{
    uint32_t off;

    if(ch > 15)
    {
        return;
    }

    if(pulse_us < PCA9685_SERVO_MIN_US)
        pulse_us = PCA9685_SERVO_MIN_US;
    if(pulse_us > PCA9685_SERVO_MAX_US)
        pulse_us = PCA9685_SERVO_MAX_US;

    /* PRESCALE=122时周期约20152us，按实际周期换算连续舵机脉宽 */
    off = ((uint32_t)pulse_us * 4096UL + 10076UL) / 20152UL;
    if(off >= 4096UL)
        off = 4095UL;

    PCA9685_SetPWM(ch, 0, (uint16_t)off);
}

/* ---------- 关闭单通道输出，用于判断舵机是否响应PWM信号 ---------- */
void PCA9685_SetChannelOff(uint8_t ch)
{
    if(ch > 15)
    {
        return;
    }

    /* LEDn_OFF_H的bit4为FULL_OFF，输出保持关闭 */
    PCA9685_SetPWM(ch, 0, 0x1000);
}

/* ---------- 角度设置：保持原工程0～180°映射，避免改变已调角度 ---------- */
void PCA9685_SetAngle(uint8_t ch, uint8_t angle)
{
    uint16_t off;

    if(angle > 180) angle = 180;
    off = 102 + (uint32_t)angle * 410 / 180;
    PCA9685_SetPWM(ch, 0, off);
}
