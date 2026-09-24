<h1 align="center">CCtrl-V2</h1>

<p align="center">
  <strong>六轴机械臂关节空间遥操作器</strong><br>
  前两连杆等长 · 支持 XZY/ZXZ 腕部姿态表示
</p>

<p align="center">
  <a href="README.md">简体中文</a> · <a href="README_EN.md">English</a>
</p>

<p align="center">
  <img alt="Firmware" src="https://img.shields.io/badge/Firmware-PlatformIO-0A66C2?style=for-the-badge">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-2EA043?style=for-the-badge">
  <img alt="Output" src="https://img.shields.io/badge/Output-RS232-7A3CF0?style=for-the-badge">
</p>

<p align="center">
  <img alt="CCtrl-V2 模型效果图" src="hardware/mechanical/renders/CCtrl-v2-cover.png">
</p>

CCtrl-V2 以六个磁编码器采集 A1–A6 关节角，手柄节点采集摇杆、按键与磁感应扳机。ESP32-S3 主控负责节点轮询、校准、OLED 菜单、腕部姿态表示转换与 RS232 输出。本仓库包含嵌入式固件、四板 JLCEDA Pro 工程、机械模型、用户文档和调试工具。

<h5>▌快速入口</h5>

| 模块 | 中文 | English |
|---|---|---|
| 用户手册 | [CCtrl-v2_User_Manual_ZHCN](docs/CCtrl-v2_User_Manual_ZHCN.md) | [CCtrl-v2_User_Manual_EN](docs/CCtrl-v2_User_Manual_EN.md) |
| 嵌入式固件 | [CCtrl-v2_Embedded_Firmware_ZHCN](docs/CCtrl-v2_Embedded_Firmware_ZHCN.md) | [CCtrl-v2_Embedded_Firmware_EN](docs/CCtrl-v2_Embedded_Firmware_EN.md) |
| 硬件工程资料 | [CCtrl-v2_Hardware_Engineering_ZHCN](docs/CCtrl-v2_Hardware_Engineering_ZHCN.md) | [CCtrl-v2_Hardware_Engineering_EN](docs/CCtrl-v2_Hardware_Engineering_EN.md) |
| 机械模型 | [CCtrl-v2_Mechanical_Models_ZHCN](docs/CCtrl-v2_Mechanical_Models_ZHCN.md) | [CCtrl-v2_Mechanical_Models_EN](docs/CCtrl-v2_Mechanical_Models_EN.md) |
| 调试工具 | [CCtrl-v2_Debug_Tools_ZHCN](docs/CCtrl-v2_Debug_Tools_ZHCN.md) | [CCtrl-v2_Debug_Tools_EN](docs/CCtrl-v2_Debug_Tools_EN.md) |

专题文档：[节点协议](docs/PROTOCOL_V2.md) · [RS232 串口协议](docs/SERIAL_PROTOCOL_ZHCN.md) · [六轴校准](docs/CALIBRATION_ZHCN.md) · [USB 调试协议](docs/USB_DEBUG_ZHCN.md)

<h5>▌项目概览</h5>

- ESP32-S3 主板通过 250000 bps 菊花链串口连接六个 Encoder 节点与一个 Handle 节点；ATmega328P 与 CH32V006 提供节点固件实现。
- 六路 AS5600 编码器按物理链路顺序映射到 A1–A6。Handle 提供摇杆、S1–S3 按键和磁感应扳机，扳机行程生成 S4。
- RS232 约 30 Hz 输出固定 39 字节 RoboMaster `0x0302` 帧；USB CDC 用于手动启动的本地调试与校准。
- OLED 菜单提供六轴校准、休息位姿保护及腕部 XZY/ZXZ 表示选择。
- 控制端输出关节计数与状态；机械臂接收端实现目标机构的尺寸、零位、方向与运动控制映射。

<h5>▌与原版 CCtrl 的关系</h5>

本项目继承 [GVD20/CCtrl](https://github.com/GVD20/CCtrl) 的 MIT 许可、主板设计和 RoboMaster `0x0302` 外层帧，重构了机构构型、节点链路与载荷语义。

| 项目 | 原版 CCtrl | CCtrl-V2 |
|---|---|---|
| 机构与传感 | 三路编码器、IMU 的空间交互机构 | 面向前两段连杆等长机械臂的六关节编码器机构 |
| 节点链路 | 原版节点协议 | Daisy-Chain Protocol V2，支持 Encoder 与 Handle 节点 |
| 对外数据 | 融合位置、姿态与滚轮等数据 | 六关节计数、手柄输入及状态；[30 字节 V2 载荷](docs/SERIAL_PROTOCOL_ZHCN.md) |
| 上位机 | WebXR 等原版工作流 | RS232 通道监视器与本地 USB 调试校准页 |

两版共用 `0x0302` 命令号；接收端按对应版本的载荷格式解析。

<h5>▌目录结构</h5>

```text
.
├─ include/
├─ lib/
├─ src/
│  ├─ master/
│  ├─ node_avr/
│  ├─ node_ch32v006/
│  ├─ shared/
│  └─ ui_runtime/
├─ test/
├─ tools/
│  ├─ rs232_channel_monitor.py
│  └─ usb_robot_debug/
├─ docs/
├─ hardware/
│  ├─ electronics/jlc_eda_pro/
│  └─ mechanical/
├─ platformio.ini
├─ LICENSE
├─ README.md
└─ README_EN.md
```

<h5>▌固件构建</h5>

```bash
pio run -e master_esp32s3
pio run -e encoder_node_atmega328p
pio run -e encoder_node_ch32v006
```

CH32V006 节点按上电时检测到的传感器进入 Encoder 或 Handle 模式；ATmega328P 节点用于 Encoder。各环境的编程协议见[嵌入式固件](docs/CCtrl-v2_Embedded_Firmware_ZHCN.md)。

<h5>▌上位机与调试</h5>

```bash
python tools/rs232_channel_monitor.py
python tools/usb_robot_debug/server.py
```

- [RS232 通道监视器](tools/RS232_CHANNEL_MONITOR_ZHCN.md)：显示 39 字节业务帧中的六轴、状态、按键和手柄数据，并绘制曲线、导出 CSV。
- [USB 调试页面](tools/usb_robot_debug/README.md)：显示原始采样、XZY/ZXZ、诊断和校准配置；业务帧仍由 RS232 输出。

<h5>▌硬件资料</h5>

电子设计以包含主板、手柄模块 v2、带滑环的编码器节点板及滑环电刷的 [JLCEDA Pro 工程](hardware/electronics/jlc_eda_pro/CCtrl-v2.epro2)提供。机械资料包含[完整装配 STEP](hardware/mechanical/cad/CCtrl-v2-assembly.step)、[局部改进替换件 STEP](hardware/mechanical/cad/CCtrl-v2-replacement-parts.step)与透明封面图。

<h5>▌PIO 库依赖</h5>

- `encoder_node_atmega328p`：`robtillaart/AS5600`
- `master_esp32s3`：`frankboesing/FastCRC`、`olikraus/U8g2`
- 本地库：`lib/WouoUiLiteGeneralBridge`、`lib/WouoUiLiteGeneralOfficial`

<h5>▌验证</h5>

三个 PlatformIO 固件环境均完成构建；协议解析、休息位姿、腕部转换和 USB 帧解析通过源码测试。

<h5>▌UI 框架说明</h5>

项目 UI 实现参考并适配了 [RQNG/WouoUI](https://github.com/RQNG/WouoUI)。

<h5>▌开源协议</h5>

本项目采用 MIT License，详见 [LICENSE](LICENSE)。
