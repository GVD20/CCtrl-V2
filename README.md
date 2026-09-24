# CCtrl-V2

![CCtrl-v2 主图](hardware/mechanical/renders/CCtrl-v2-cover.png)

[English](README_EN.md) · [用户手册](docs/CCtrl-v2_User_Manual_ZHCN.md) · [串口协议](docs/SERIAL_PROTOCOL_ZHCN.md)

CCtrl-v2 是面向**前两段连杆等长的六自由度串联机械臂**的关节空间遥操作控制器。六个磁编码器采集 A1–A6 关节角，手柄节点采集摇杆、按键与磁感应扳机；ESP32-S3 主控负责节点轮询、校准、OLED 菜单、腕部姿态表示转换及输出。控制端提供关节数据，机械臂侧实现与目标机构尺寸、零位、方向和运动范围对应的映射与控制。

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
| 用户手册 | [中文](docs/CCtrl-v2_User_Manual_ZHCN.md) · [English](docs/CCtrl-v2_User_Manual_EN.md) |
| 嵌入式固件 | [中文](docs/CCtrl-v2_Embedded_Firmware_ZHCN.md) · [English](docs/CCtrl-v2_Embedded_Firmware_EN.md) |
| 硬件工程资料 | [中文](docs/CCtrl-v2_Hardware_Engineering_ZHCN.md) · [English](docs/CCtrl-v2_Hardware_Engineering_EN.md) |
| 机械模型 | [中文](docs/CCtrl-v2_Mechanical_Models_ZHCN.md) · [English](docs/CCtrl-v2_Mechanical_Models_EN.md) |
| 调试工具 | [中文](docs/CCtrl-v2_Debug_Tools_ZHCN.md) · [English](docs/CCtrl-v2_Debug_Tools_EN.md) |
| 协议与校准细节 | [节点协议](docs/PROTOCOL_V2.md) · [RS232 串口协议](docs/SERIAL_PROTOCOL_ZHCN.md) · [六轴校准](docs/CALIBRATION_ZHCN.md) · [USB 调试协议](docs/USB_DEBUG_ZHCN.md) |

## 固件构建

`pio run` 构建 PlatformIO 配置中的三个环境：`master_esp32s3`、`encoder_node_atmega328p` 和 `encoder_node_ch32v006`。CH32V006 节点在上电时根据传感器识别结果进入编码器或手柄模式；ATmega328P 环境用于编码器节点。各环境的编程协议见[嵌入式固件](docs/CCtrl-v2_Embedded_Firmware_ZHCN.md)。

## 运行结构

六个编码器节点按链路中的物理顺序映射为 A1–A6，手柄节点使用独立字段。OLED 的 `Monitor` 与 `Debug` 显示节点、磁场和通信状态；`Calibration → Encoder_calibration` 保存六轴校准。RS232 按固定 39 字节帧发送关节与手柄数据。[通道监视器](tools/RS232_CHANNEL_MONITOR_ZHCN.md)显示这些字段，[USB 调试页](tools/usb_robot_debug/README.md)显示原始采样、双腕部表示和配置。完整定义见[用户手册](docs/CCtrl-v2_User_Manual_ZHCN.md)。

## 硬件资料

电子设计以包含四块板的 JLCEDA Pro 工程提供，制造文件可在 JLCEDA Pro 中导出。机械资料包含完整装配 STEP、局部改进替换件 STEP 和透明封面图。替换件与完整装配中的对应零件通过模型几何和装配位置关联。

## 验证

三个 PlatformIO 固件环境均完成构建；协议解析、休息位姿、腕部转换和 USB 帧解析通过源码测试。

## 许可

项目沿用 [MIT License](LICENSE)。UI 实现参考 [RQNG/WouoUI](https://github.com/RQNG/WouoUI)。
