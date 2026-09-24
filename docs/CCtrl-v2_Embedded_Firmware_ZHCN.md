# CCtrl-V2 嵌入式固件

[ZHCN](CCtrl-v2_Embedded_Firmware_ZHCN.md) | [EN](CCtrl-v2_Embedded_Firmware_EN.md)

本页汇总 CCtrl-V2 的固件环境、源码入口和运行职责。

## 主要文件与目录

| 入口 | 内容 |
|---|---|
| [platformio.ini](../platformio.ini) | 三个 PlatformIO 环境、依赖与编程协议 |
| [src/master](../src/master) | ESP32-S3 主控业务、校准、腕部转换、USB 调试 |
| [src/node_avr](../src/node_avr) | ATmega328P 编码器节点 |
| [src/node_ch32v006](../src/node_ch32v006) | CH32V006 Encoder/Handle 节点 |
| [src/shared](../src/shared) | 菊花链帧、CRC 与公共协议定义 |
| [src/ui_runtime](../src/ui_runtime) | OLED UI 运行入口 |
| [include](../include)、[lib](../lib) | UI 配置、业务接口与本地库 |
| [test](../test) | 协议、休息位姿与腕部表示测试 |

## 构建命令

| 环境 | 芯片与用途 | 命令 |
|---|---|---|
| `master_esp32s3` | ESP32-S3 主控 | `pio run -e master_esp32s3` |
| `encoder_node_atmega328p` | 8 MHz ATmega328P Encoder | `pio run -e encoder_node_atmega328p` |
| `encoder_node_ch32v006` | CH32V006 Encoder/Handle | `pio run -e encoder_node_ch32v006` |

配置中的主控编程协议为 esptool，ATmega328P 为 USBasp，CH32V006 为 WCH-Link。PlatformIO 上传目标为 `pio run -e <环境名> -t upload`。

## 运行结构

主控使用 250000 bps 菊花链链路枚举和轮询节点，约 45 Hz 收集采样。RS232 以 115200 bps、约 30 Hz 输出 39 字节业务帧。CH32V006 节点按上电时检测到的传感器进入 Encoder 或 Handle 模式；ATmega328P 环境实现 Encoder 节点。OLED 配置中的六轴校准、腕部表示和休息位姿写入 ESP32 NVS。

## 相关文档

- [用户手册](CCtrl-v2_User_Manual_ZHCN.md)
- [Daisy-Chain Protocol V2](PROTOCOL_V2.md)
- [RS232 串口协议](SERIAL_PROTOCOL_ZHCN.md)
- [六轴校准](CALIBRATION_ZHCN.md)
