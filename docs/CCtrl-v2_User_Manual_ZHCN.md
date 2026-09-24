# CCtrl-V2 用户手册

[ZHCN](CCtrl-v2_User_Manual_ZHCN.md) | [EN](CCtrl-v2_User_Manual_EN.md)

本文档描述 CCtrl-V2 的组成、运行结构、通信数据、设备配置和工具入口。字段的完整字节定义见[串口协议](SERIAL_PROTOCOL_ZHCN.md)，节点链路定义见[Daisy-Chain Protocol V2](PROTOCOL_V2.md)。

## 1. 项目定义

CCtrl-V2 是面向前两段连杆等长的六自由度串联机械臂的关节空间遥操作控制器。六个磁编码器采集 A1–A6 关节位置；手柄节点采集摇杆、S1–S3 按键和磁感应扳机。ESP32-S3 主控完成节点枚举与轮询、校准、OLED 显示、腕部姿态表示转换和对外输出。

系统输出的是控制器关节计数、输入设备数据和链路状态。目标机械臂的连杆尺寸、关节零位、传动比例、运动学和电机控制由机械臂接收端实现。

| 组成 | 实现 | 数据 |
|---|---|---|
| 主控 | ESP32-S3 主板 | 六轴校准值、菜单与输出状态 |
| 关节节点 | 六个 AS5600 编码器节点 | 磁场状态与一圈 4096 计数的原始角度 |
| 手柄节点 | Handle 模块 | 摇杆、按键和磁场 XYZ |
| 对外接口 | RS232 | 约 30 Hz 的 RoboMaster `0x0302` 帧 |
| 本地调试接口 | USB CDC | 手动启动的校准与状态快照 |

## 2. 固件与运行结构

- [`src/master`](../src/master)：ESP32-S3 的总线轮询、数据整合、校准、RS232 和 USB 调试实现。
- [`src/node_avr`](../src/node_avr)：ATmega328P 编码器节点固件。
- [`src/node_ch32v006`](../src/node_ch32v006)：CH32V006 节点固件，上电后按检测到的传感器进入 Encoder 或 Handle 模式。
- [`src/shared`](../src/shared)：Daisy-Chain Protocol V2 与 CRC 公共实现。
- [`src/ui_runtime`](../src/ui_runtime)、[`include`](../include)、[`lib`](../lib)：OLED UI、公共接口和本地库。

主控按物理链路顺序枚举节点。Encoder 出现的先后顺序映射为 A1–A6；Handle 有独立的数据字段，处于编码器节点之间时也不改变轴号。总线轮询周期约 22.2 ms，业务帧输出周期约 33.3 ms。

## 3. 通信链路

### 3.1 对外链路

ESP32-S3 通过 RS232 以 115200 bps、8N1 向外部控制器单向输出业务帧。帧由 `0xA5` 帧头、长度、序号、CRC8、命令号 `0x0302`、30 字节载荷和 CRC16 组成，总长 39 字节。RS232 输出与 USB 调试各自使用独立的数据通道。

### 3.2 主控与节点链路

主控与节点之间的菊花链串口速率为 250000 bps。`ENUM_RESET (0x01)` 根据物理顺序分配地址，`POLL (0x02)` 逐节点收集记录。节点记录包含 ID、类型、平台和节点数据；Encoder 提供状态与原始角度，Handle 提供摇杆、磁场 XYZ 与按键。帧格式、节点状态位和 LED 行为见[节点协议](PROTOCOL_V2.md)。

### 3.3 USB 调试链路

USB CDC 上电时处于关闭状态。OLED 主菜单的 `USB Debug → Start Debug` 启动串口，设备名为 `CCtrl USB Debug`，网页以 2000000 baud 连接。主控约 15 Hz 发送原始六轴、XZY、ZXZ、诊断与配置快照。USB 调试串口保持开启至设备重启；业务帧继续通过 RS232 输出。[USB 调试协议](USB_DEBUG_ZHCN.md)列出帧类型与配置命令。

## 4. 统一输出协议（0x0302）

### 4.1 帧结构

```text
SOF(0xA5) + data_length(2) + sequence(1) + CRC8(1)
+ cmd_id(0x0302, 2) + payload(30) + CRC16(2)
```

多字节整数采用小端序。`data_length` 固定为 30；`sequence` 为模 256 递增的单字节帧序号。

### 4.2 30 字节载荷定义

| 偏移 | 长度 | 字段 | 类型 | 内容 |
|---:|---:|---|---|---|
| 0 | 1 | `diagnostic_flags` | `uint8` | 系统状态、超时和警告 |
| 1 | 1 | `node_valid_flags` | `uint8` | A1–A6 与 Handle 有效位 |
| 2 | 1 | `main_buttons` | `uint8` | 主控 KEY1–KEY4 单击、双击事件 |
| 3 | 1 | `handle_buttons` | `uint8` | S1–S4 状态 |
| 4–9 | 6 | `encoder_status[6]` | `uint8[6]` | 六个编码器的采样状态 |
| 10–21 | 12 | `encoder_value[6]` | `int16[6]` | 六个校准后的关节计数 |
| 22–27 | 6 | `joystick_x/y`, `trigger` | `uint16[3]` | 手柄模拟量，范围 0–4095 |
| 28–29 | 2 | `reserved` | `uint16` | 当前为 0 |

### 4.3 状态与有效位

`diagnostic_flags` 的 bit0–1 表示 `INIT`、`ACTIVE`、`DEGRADED`、`DISCONNECTED`；bit2 表示链路超时；bit3–6 依次表示 CRC、格式、磁场和节点数量警告。`node_valid_flags` 的 bit0–5 对应 A1–A6，bit6 对应 Handle。采样中断时数值字段保留最近一次有效值，有效位与诊断字段反映当前状态。

### 4.4 关节计数

每轴校准后的范围为 `-2048…2047`，一圈为 4096 计数。角度为 `encoder_value × 360 / 4096` 度。校准零位的正对侧是 `2047/-2048` 环形折返点。`encoder_status` 的 bit0 表示 I2C 通信、bit1 表示检测到磁铁、bit2/3 表示磁场过弱/过强、bit4 表示数据过期。正常且检测到磁铁时状态值为 `0x03`。

完整字段、CRC 算法和示例帧见[串口协议](SERIAL_PROTOCOL_ZHCN.md)。两版 CCtrl 均使用命令号 `0x0302`，CCtrl-V2 的接收格式由本节的 30 字节载荷定义。

## 5. 输出语义

### 5.1 六轴与腕部

A1–A3 为主体关节，A4–A6 为腕部关节。OLED `Options → Wrist` 的 `XZY` 模式直接输出校准后的 A4–A6；`ZXZ` 模式将同一腕部姿态转换为另一组欧拉角。转换约定为 `Rx(-A4)·Rz(A5)·Ry(-A6) = Rz(α)·Rx(β)·Rz(γ)`。ZXZ 在 β 接近 0° 或 180° 时使用连续分支跟踪；腕部表示的选择保存在 NVS 中。

### 5.2 按键与模拟量

`main_buttons` 低四位是 KEY1–KEY4 的单击事件，高四位是同一组物理键的双击事件。固件使用 20 ms 去抖、300 ms 双击窗口、160 ms 事件脉冲和 200 ms 同键不应期。`handle_buttons` 的 bit0–2 对应 S1–S3；bit3 为扳机产生的 S4，行程达到 80% 时置位，降到 50% 以下时清除。

摇杆 X/Y 与扳机范围为 0–4095，摇杆中心为 2048。主控对摇杆原始 ADC 值进行分段线性映射，Y 轴在映射后反向。校准常数见[Handle 摇杆校准](JOYSTICK_CALIBRATION_ZHCN.md)。

### 5.3 上电休息位姿

`Options → RestGuard` 保存下次上电的保护开关。保护启用时，设备进入已保存的六轴休息区域并稳定 500 ms 之前，RS232 输出保存的标准休息位姿，系统状态为 `DEGRADED`。A1、A5、A6 的进入/退出阈值为 ±30°/±35°；A2–A4 为 ±2°/±3°。位置条件与连续 500 ms 时间条件共同决定进入和退出。USB 页面保存当前六轴健康、新鲜的原始读数作为休息位姿。

## 6. 设备侧配置与校准

OLED `Calibration → Encoder_calibration` 包含 `Zero All 6`、A1–A6 的 `Reverse`、`Save` 与 `Reload`。`Zero All 6` 采集六路 Encoder 当前原始值，将当前位置设为下表的校准角。六路节点存在、I2C 正常且检测到磁铁时，固件将整组校准数据写入 NVS。

| 轴 | `Zero All 6` 后当前位置 |
|---|---:|
| A1 | 0° |
| A2 | +20°，约 228 计数 |
| A3 | +60°，约 683 计数 |
| A4–A6 | 0° |

校准配置包含六路 offset 与 direction，以带版本和 CRC16 的完整 NVS Blob 保存。USB 调试页面显示原始计数、偏移、方向和补偿角度，并提供 0°、±90°、180° 与自定义单轴目标；页面通过带修订号与 CRC 的事务保存整组六轴配置。计算方法见[六路 Encoder 校准](CALIBRATION_ZHCN.md)。

## 7. 上位机工具

- [RS232 通道监视器](../tools/RS232_CHANNEL_MONITOR_ZHCN.md)：`python tools/rs232_channel_monitor.py`。显示协议字段、状态与曲线，并导出当前时间窗口的原始值 CSV。
- [USB Web Serial 调试器](../tools/usb_robot_debug/README.md)：`python tools/usb_robot_debug/server.py`。本地页面位于 `http://127.0.0.1:8768`，显示原始六轴、XZY/ZXZ、诊断、校准与休息位姿。
- [协议参考解析](../tools/protocol_v2.py)：Python 实现的 RS232 帧解析与 CRC 校验。

工具目录与功能见[调试工具](CCtrl-v2_Debug_Tools_ZHCN.md)。

## 8. 构建与烧录

[`platformio.ini`](../platformio.ini) 定义三个固件环境：

| 环境 | 硬件 | 编程协议 | 构建命令 |
|---|---|---|---|
| `master_esp32s3` | ESP32-S3 主板 | esptool | `pio run -e master_esp32s3` |
| `encoder_node_atmega328p` | 8 MHz ATmega328P 编码器节点 | USBasp | `pio run -e encoder_node_atmega328p` |
| `encoder_node_ch32v006` | CH32V006 节点 | WCH-Link | `pio run -e encoder_node_ch32v006` |

PlatformIO 的上传目标为 `pio run -e <环境名> -t upload`。固件文件、依赖和运行职责见[嵌入式固件](CCtrl-v2_Embedded_Firmware_ZHCN.md)。

## 9. 项目资料与验证

| 资料 | 内容 |
|---|---|
| [硬件工程资料](CCtrl-v2_Hardware_Engineering_ZHCN.md) | 主板、手柄模块 v2、带滑环的编码器节点板和滑环电刷的 JLCEDA Pro 工程 |
| [机械模型](CCtrl-v2_Mechanical_Models_ZHCN.md) | 完整装配 STEP、局部改进替换件 STEP 和透明封面图 |
| [节点协议](PROTOCOL_V2.md) | 枚举、轮询、节点记录、状态与 LED |
| [对外串口协议](SERIAL_PROTOCOL_ZHCN.md) | 39 字节帧、30 字节载荷与字段解释 |

项目源码的三个 PlatformIO 环境已完成构建。协议解析测试 12 项、休息位姿与腕部转换的 C++ 测试，以及 USB 帧解析自检均已通过。
