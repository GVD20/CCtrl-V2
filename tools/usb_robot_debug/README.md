# CCtrl-v2 Web Serial 调试器

ESP32-S3 主机的本地 USB 校准与双姿态监视页面。每次上电 USB 调试均关闭：先在
主菜单进入 `USB Debug`，选择 `Start Debug`，再连接数据线。启动后持续
开启，需要关闭时重启设备；一般业务始终只通过 RS232 输出。

```powershell
python tools\usb_robot_debug\server.py
```

然后使用支持 Web Serial 的 Chrome 或 Edge 打开 <http://127.0.0.1:8768>。

ESP32 在 30 Hz 输出节拍中同时计算 XZY 和 ZXZ，USB 调试约 15 Hz 发送最近快照；
页面直接展示设备生成的两组结果。RS232 按选定的 Wrist 格式输出。

诊断区以8位显示主机快捷键事件：低4位为KEY1–KEY4单击，高4位为对应双击
KEY5–KEY8。
