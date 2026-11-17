#include "usart.h"
#include "comm.h"
#include "wifi.h"

struct __FILE 
{ 
	int handle; /* Add whatever you need here */ 
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) 
{
	/* 发送数据 */
	USART_SendData(USART1, ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	return ch;
}

//USART1	TX1	PA9
//USART1	RX1	PA10
void init_usart1(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能端口A的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//使能串口1的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 配置串口的TX（PA9）跟RX（PA10）引脚复用 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//工作模式：复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//高速
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//端口A引脚9配置为复用功能模式
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	//端口A引脚10配置为复用功能模式
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//串口硬件配置：波特率，数据位，停止位，校验位，流控
	USART_InitStructure.USART_BaudRate = baud;						//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//接收 | 发送模式
	USART_Init(USART1, &USART_InitStructure);
	
	/* 使能串口接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* NVIC configuration */
	/* 使能 USART1 中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 使能 USART1 工作*/
	USART_Cmd(USART1, ENABLE);
}

void send_char(uint16_t ch)
{
	/* 发送数据 */
	USART_SendData(USART1, ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void send_str(char *str)
{
	while(*str != '\0')
	{
		send_char(*str);
		str++;
	}
}

void USART1_IRQHandler(void)
{
    uint8_t ch;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART1);
        Comm_RxByteFromISR(COMM_SOURCE_SERIAL, ch);
    }

    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE |
                           USART_FLAG_FE | USART_FLAG_NE |
                           USART_FLAG_PE) != RESET)
    {
        (void)USART_ReceiveData(USART1);
    }
}

//------------------------------------------------------------------------------------------------------------
void init_usart2(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能端口A的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//使能串口2的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* 配置串口的TX（PA2）跟RX（PA3）引脚复用 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//工作模式：复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//高速
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//端口A引脚2配置为复用功能模式
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	//端口A引脚3配置为复用功能模式
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//串口硬件配置：波特率，数据位，停止位，校验位，流控
	USART_InitStructure.USART_BaudRate = baud;						//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//接收 | 发送模式
	USART_Init(USART2, &USART_InitStructure);
	
	/* 使能串口接收中断 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* NVIC configuration */
	/* 使能 USART2 中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 使能 USART2 工作*/
	USART_Cmd(USART2, ENABLE);
}

void send2_char(uint16_t ch)
{
	/* 发送数据 */
	USART_SendData(USART2, ch);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void send2_str(char *str)
{
	while(*str != '\0')
	{
		send2_char(*str);
		str++;
	}
}

//编写 中断服务函数（用来接收数据）
void USART2_IRQHandler(void)
{
    uint8_t ch;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART2);
        Comm_RxByteFromISR(COMM_SOURCE_BLUETOOTH, ch);
    }

    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE |
                           USART_FLAG_FE | USART_FLAG_NE |
                           USART_FLAG_PE) != RESET)
        (void)USART_ReceiveData(USART2);
}

//----------------------------------------------------------------------------------------------------------

void init_usart3(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能端口B的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//使能串口3的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* 配置串口的TX（PB10）跟RX（PB11）引脚复用 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//工作模式：复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//高速
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//端口B引脚10配置为复用功能模式
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	//端口B引脚11配置为复用功能模式
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//串口硬件配置：波特率，数据位，停止位，校验位，流控
	USART_InitStructure.USART_BaudRate = baud;						//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//接收 | 发送模式
	USART_Init(USART3, &USART_InitStructure);
	
	/* 使能串口接收中断 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* NVIC configuration */
	/* 使能 USART3 中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 使能 USART3 工作*/
	USART_Cmd(USART3, ENABLE);
}

void send3_char(uint16_t ch)
{
	/* 发送数据 */
	USART_SendData(USART3, ch);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void send3_str(char *str)
{
	while(*str != '\0')
	{
		send3_char(*str);
		str++;
	}
}

//编写 中断服务函数（用来接收数据）
void USART3_IRQHandler(void)
{
    uint8_t ch;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART3);
        if(WiFi_IsConnected())
            Comm_RxByteFromISR(COMM_SOURCE_WIFI, ch);
        else
            WiFi_RxByteFromISR(ch);
    }

    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE |
                           USART_FLAG_FE | USART_FLAG_NE |
                           USART_FLAG_PE) != RESET)
        (void)USART_ReceiveData(USART3);
}

//-----------------------------------------------------------------------------------

//U6_TX	PC6	camera   8
//U6_RX	PC7	camera   9

void init_usart6(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能端口C的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	//使能串口6的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	/* 配置串口的TX（PC6）跟RX（PC7）引脚复用 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//工作模式：复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//高速
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//端口C引脚6配置为复用功能模式
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

	//端口C引脚7配置为复用功能模式
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	
	//串口硬件配置：波特率，数据位，停止位，校验位，流控
	USART_InitStructure.USART_BaudRate = baud;						//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//接收 | 发送模式
	USART_Init(USART6, &USART_InitStructure);
	
	/* 使能串口接收中断 */
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	/* NVIC configuration */
	/* 使能 USART6 中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 使能 USART6 工作*/
	USART_Cmd(USART6, ENABLE);
}

void send6_char(uint16_t ch)
{
	/* 发送数据 */
	USART_SendData(USART6, ch);
	while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
}

void send6_str(char *str)
{
	while(*str != '\0')
	{
		send6_char(*str);
		str++;
	}
}

//编写 中断服务函数（用来接收数据）
void USART6_IRQHandler(void)
{
    uint8_t ch;

    if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART6);
        Comm_RxByteFromISR(COMM_SOURCE_SERIAL, ch);
    }

    if(USART_GetFlagStatus(USART6, USART_FLAG_ORE |
                           USART_FLAG_FE | USART_FLAG_NE |
                           USART_FLAG_PE) != RESET)
        (void)USART_ReceiveData(USART6);
}



