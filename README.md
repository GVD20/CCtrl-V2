# CCtrl-v2

![CCtrl-v2 主图](hardware/mechanical/renders/CCtrl-v2-cover.png)

[English](README_EN.md) · [用户手册](docs/CCtrl-v2_User_Manual_ZHCN.md) · [串口协议](docs/SERIAL_PROTOCOL_ZHCN.md)

CCtrl-v2 是面向**前两段连杆等长的六自由度串联机械臂**的关节空间遥操作控制器。六个磁编码器采集 A1–A6 关节角，手柄节点采集摇杆、按键与磁感应扳机；ESP32-S3 主控负责节点轮询、校准、OLED 菜单、腕部姿态表示转换及输出。控制端提供关节数据，机械臂侧需按自身尺寸、零位、方向和安全限制完成映射与控制。

## 与原版 CCtrl 的关系

本项目继承 [GVD20/CCtrl](https://github.com/GVD20/CCtrl) 的 MIT 许可、主板设计和 RoboMaster `0x0302` 外层帧。新版重构了机构构型、节点链路和载荷语义：

| 项目 | 原版 CCtrl | CCtrl-v2 |
|---|---|---|
| 机构与传感 | 三路编码器、IMU 的空间交互机构 | 面向前两段连杆等长机械臂的六关节编码器机构 |
| 节点链路 | 原版节点协议 | 兼容两种异构节点，支持新版手柄 |
| 对外数据 | 融合位置、姿态与滚轮等数据 | 六关节计数、手柄输入及状态；使用 [30 字节 V2 载荷](docs/SERIAL_PROTOCOL_ZHCN.md) |
| 上位机 | WebXR 等原版工作流 | RS232 通道监视器与本地 USB 调试校准页 |

两版共用 `0x0302` 命令号；接收端按对应版本的载荷格式解析。

## 仓库入口

| 内容 | 入口 |
|---|---|
| 使用、校准、故障排查 | [CCtrl-v2 用户手册](docs/CCtrl-v2_User_Manual_ZHCN.md) |
| 主控与节点协议 | [Protocol V2](docs/PROTOCOL_V2.md) |
| 对外 RS232 帧 | [串口协议](docs/SERIAL_PROTOCOL_ZHCN.md) |
| 六轴校准与 USB 调试 | [校准](docs/CALIBRATION_ZHCN.md) · [USB 调试](docs/USB_DEBUG_ZHCN.md) |
| 电子工程 | [硬件说明](hardware/README.md) · [JLCEDA Pro 工程](hardware/electronics/jlc_eda_pro/CCtrl-v2.epro2) |
| 机械模型 | [模型说明](hardware/mechanical/README.md) |
| RS232 监视器 | [工具说明](tools/RS232_CHANNEL_MONITOR_ZHCN.md) |
| USB 校准页面 | [工具说明](tools/usb_robot_debug/README.md) |

## 固件构建

使用 PlatformIO，在仓库根目录执行 `pio run`。三个环境分别为 `master_esp32s3`、`encoder_node_atmega328p` 和 `encoder_node_ch32v006`。CH32V006 节点在上电时根据传感器识别结果进入编码器或手柄模式；ATmega328P 环境用于编码器节点。烧录端口与编程器按本机硬件选择，参见[用户手册](docs/CCtrl-v2_User_Manual_ZHCN.md)。

## 从装配到联调

1. 用 JLCEDA Pro 打开四板工程，在 CAD 中查看完整装配与改进替换件，确认板间连接、滑环通道和运动间隙。
2. 构建并写入主板与节点固件，按物理链路顺序连接六个编码器节点及手柄节点。
3. 上电后在 OLED 的 `Monitor`、`Debug` 中核对节点数量、磁场状态和六轴有效位。
4. 将机构摆到基准姿态，通过 `Calibration → Encoder_calibration` 执行六轴校准，再逐轴检查方向。
5. 用 [RS232 通道监视器](tools/RS232_CHANNEL_MONITOR_ZHCN.md)检查 39 字节业务帧与摇杆、扳机、按键；需要逐轴目标角或休息位姿时，启动[USB 调试页](tools/usb_robot_debug/README.md)。
6. 机械臂接收端按[本版串口协议](docs/SERIAL_PROTOCOL_ZHCN.md)解析关节计数、有效位与状态，并完成目标机构的映射和控制。

完整操作、校准原理和故障排查见[用户手册](docs/CCtrl-v2_User_Manual_ZHCN.md)。

## 硬件资料

电子设计以包含四块板的 JLCEDA Pro 工程提供，制造文件可在 JLCEDA Pro 中导出。机械资料包含完整装配 STEP、局部改进替换件 STEP 和透明封面图。装配时需在 CAD 中核对替换件对应关系及配合尺寸。

## 验证

三个 PlatformIO 固件环境均完成构建；协议解析、休息位姿、腕部转换和 USB 帧解析通过源码测试。

## 许可

项目沿用 [MIT License](LICENSE)。UI 实现参考 [RQNG/WouoUI](https://github.com/RQNG/WouoUI)。
