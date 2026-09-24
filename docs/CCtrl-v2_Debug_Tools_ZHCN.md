# CCtrl-V2 调试工具

[ZHCN](CCtrl-v2_Debug_Tools_ZHCN.md) | [EN](CCtrl-v2_Debug_Tools_EN.md)

本页汇总仓库中的 RS232 监视器、USB 调试页面和协议参考实现。

## 主要文件与目录

| 入口 | 功能 |
|---|---|
| [rs232_channel_monitor.py](../tools/rs232_channel_monitor.py) | 解析 RS232 `0x0302` 帧，显示字段和曲线，导出 CSV |
| [start_rs232_channel_monitor.bat](../tools/start_rs232_channel_monitor.bat) | Windows 监视器启动入口 |
| [usb_robot_debug](../tools/usb_robot_debug) | Web Serial 本地页面、帧解析、校准及休息位姿操作 |
| [protocol_v2.py](../tools/protocol_v2.py) | Python 帧解析与 CRC 参考实现 |

## 启动方式

| 工具 | 命令 | 接口 |
|---|---|---|
| RS232 通道监视器 | `python tools/rs232_channel_monitor.py` | 115200 8N1 的 RS232 转接串口 |
| USB 调试页面 | `python tools/usb_robot_debug/server.py` | `http://127.0.0.1:8768`；浏览器 Web Serial，2000000 baud |

RS232 通道监视器以 39 字节业务帧为输入，显示六轴、有效位、状态、按键、摇杆和扳机；曲线窗口与 CSV 使用协议原始数值。USB 页面在设备的 `USB Debug → Start Debug` 开启后连接，显示约 15 Hz 的原始六轴、XZY、ZXZ 与诊断快照，并发送校准、休息位姿配置命令。

## 相关文档

- [用户手册](CCtrl-v2_User_Manual_ZHCN.md)
- [监视器说明](../tools/RS232_CHANNEL_MONITOR_ZHCN.md)
- [USB 页面说明](../tools/usb_robot_debug/README.md)
- [USB 调试协议](USB_DEBUG_ZHCN.md)
