# CCtrl-V2 硬件工程资料

[ZHCN](CCtrl-v2_Hardware_Engineering_ZHCN.md) | [EN](CCtrl-v2_Hardware_Engineering_EN.md)

本页汇总 CCtrl-V2 的电子工程文件及四块板的项目组成。

## 主要文件与目录

- [JLCEDA Pro 工程](../hardware/electronics/jlc_eda_pro/CCtrl-v2.epro2)：原理图与 PCB 的项目文件。
- [电子工程说明](../hardware/electronics/README.md)：工程格式与板卡列表。
- [硬件资料入口](../hardware/README.md)：电子与机械资料目录。

## 工程组成

| 板卡 | 项目内名称 | 用途 |
|---|---|---|
| 主板 | `CC_ESP32main` | ESP32-S3 主控与对外接口 |
| 手柄模块 v2 | `Handle_node_v2` | 摇杆、按键与磁感应扳机 |
| 带滑环的编码器节点板 | `Encoder_node_v3` | 关节角采集与菊花链转发 |
| 滑环电刷 | `Encoder_brush` | 旋转关节的电气连接 |

电子设计以 JLCEDA Pro 原生项目提供，制造输出由该工程导出。主板沿用原版 CCtrl 的设计基础，节点与手柄对应六轴机构及 V2 固件。

## 相关文档

- [用户手册](CCtrl-v2_User_Manual_ZHCN.md)
- [嵌入式固件](CCtrl-v2_Embedded_Firmware_ZHCN.md)
- [机械模型](CCtrl-v2_Mechanical_Models_ZHCN.md)
