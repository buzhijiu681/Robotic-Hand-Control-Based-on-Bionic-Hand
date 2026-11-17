# 基于仿生手的手势控制机械手

本项目使用 STM32F103 采集数据手套的五路弯曲传感器，通过 ESP8266 WiFi（或 USART1 有线链路）把手指弯曲量发送给 STM32F407。机械手端解析 `GLOVE` 数据，并通过 PCA9685 驱动六路舵机，实时复现手势；同时支持串口、蓝牙和 WiFi 三种控制入口。

```text
五路弯曲传感器
      │ ADC
      ▼
STM32F103（仿生手套端）
      │ ESP8266 / USART1
      │ GLOVE、DEMO_ONCE、CH5_ONCE
      ▼
STM32F407（机械手端）
      │ I²C
      ▼
PCA9685 ──> 0～4 号位置舵机 + 5 号连续舵机
```

## 工程分类

| 目录 | 目标芯片 | 作用 |
| --- | --- | --- |
| [`firmware/bionic_glove_stm32f103`](firmware/bionic_glove_stm32f103) | STM32F103 | 手套端关键代码：五路 ADC、标定/滤波、ESP8266 AP + TCP 服务器、USART1 有线兜底 |
| [`firmware/servo_hand_controller_stm32f407`](firmware/servo_hand_controller_stm32f407) | STM32F407 | 机械手端关键代码：五路位置舵机和一路连续舵机、三路通信、PCA9685、演示与校准命令 |

仓库只保留正式控制链路中的自研应用代码和必要的板级初始化代码，不包含测试工程、旧版本备份、芯片厂商库、Keil 工程产物或烧录文件。

## 快速开始

1. 使用 Keil MDK 和 STM32 标准外设库新建工程，分别加入两个目录中的源码。
2. 手套端使用 STM32F10x Standard Peripheral Library V3.5.0，并启用 `USE_STDPERIPH_DRIVER`。
3. 机械手端使用 STM32F4xx Standard Peripheral Library V1.4.0，并启用 `USE_STDPERIPH_DRIVER`。
4. 加入对应的 CMSIS 设备头文件、启动文件、`system_stm32fxxx.c` 和工程原有中断处理文件。
5. 检查两端 `app_config.h` 中的热点、密码、IP、端口和波特率是否一致。

两个目录中的 `README.md` 给出了文件组织、接线与运行参数。芯片厂商库体积较大，请直接使用本地原工程或 ST 官方发行包，不重复上传到本仓库。

正式固件默认的 WiFi 参数如下：

| 参数 | 默认值 |
| --- | --- |
| 热点 | `PYY_8266` |
| 密码 | `12345678` |
| TCP 服务器 | `192.168.4.1:80` |
| ESP8266 UART | `115200 8N1` |

## 主要通信协议

仿生手套端周期发送五路弯曲百分比：

```text
GLOVE,S1,S2,S3,S4,S5
GLOVE,0,50,20,80,100
```

机械手端把 S1、S2、S4、S5 映射到四路位置舵机，S3 用于触发 5 号连续舵机的一次性动作。五路全部超过阈值时还会发送 `DEMO_ONCE` 启动完整手势演示。

完整命令、接线、校准和故障排查说明请查看：

- [手套端说明](firmware/bionic_glove_stm32f103/README.md)
- [机械手端说明](firmware/servo_hand_controller_stm32f407/README.md)

## 仓库约定

- 只提交正式固件的关键源码，测试代码留在本地开发目录。
- 芯片厂商库、Keil 工程、日志、J-Link 本地配置和编译产物不提交。
- 修改硬件参数时优先修改各工程的 `app_config.h`，再重新编译。

## 许可

本仓库沿用根目录 [LICENSE](LICENSE) 中的许可条款。
