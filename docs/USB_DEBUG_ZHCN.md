# CCtrl-v2 USB 串口调试协议

## 1. 启停和业务隔离

- ESP32-S3 每次上电时 USB CDC 默认关闭。
- 从主菜单进入 `USB Debug` 并选择 `Start Debug`，随后设备枚举 USB
  调试串口。调试串口持续开启至设备重启。
- Daisy-Chain 采样仍约45 Hz；一般业务始终只从 RS232 以约30 Hz输出，帧不变。
- XZY和ZXZ在既有主机输出时点同时生成；USB以约15 Hz发送最近快照，不再次推进
  ZXZ连续跟随状态。网页只展示两份设备结果，不做轴序换算。

本地页面位于 `tools/usb_robot_debug`：

```powershell
python tools\usb_robot_debug\server.py
```

## 2. 串口帧

USB CDC 名称为 `CCtrl USB Debug`，网页以2,000,000 baud打开。所有多字节字段均为
小端。每帧格式如下：

```text
43 44 | version:u8 | type:u8 | sequence:u8 | payload_len:u8 |
payload[0..32] | crc16:u16
```

CRC16覆盖帧头和payload，不含末尾CRC本身。版本当前为1。

| type | payload |
|---:|---|
| `01` | `valid:u8, raw[6]:u16` |
| `02` | `valid:u8, xzy[6]:i16` |
| `03` | `valid:u8, zxz[6]:i16` |
| `04` | 状态、错误、警告、按键、休息标志、摇杆、扳机、延迟、丢失轮询 |
| `05` | `revision:u16, offset[6]:u16, direction_mask:u8` |
| `06` | `revision:u16, rest_raw[6]:u16, flags:u8` |
| `07` | 命令结果 |
| `80` | 网页发往ESP32的命令 |

XZY/ZXZ 均为一圈 4096 计数的有符号值；USB 页面展示设备发送的两组结果，
不自行重新计算 ZXZ。轴序和方向约定见 [串口协议](SERIAL_PROTOCOL_ZHCN.md)。

## 3. 命令

- `01`：读取校准配置和休息位姿。
- `10 revision:u16 offset[6]:u16 direction_mask:u8`：原子应用并保存六轴校准。
- `20`：将当前六路fresh、健康的原始值保存为休息位姿。
- `21 00/01`：关闭或启用下次上电的休息位姿保护。

## 4. 休息位姿保护

- A1、A5、A6进入阈值为±30°，退出阈值为±35°；A2–A4进入±2°，退出±3°。
- 六个关节均参与休息位姿判定和休息位姿记录。
- 进入和退出均须同时满足位置迟滞条件和连续500 ms时间条件。
- 保护启用时，上电持续发送保存的标准休息位姿，直到进入休息区域500 ms。
- 保护激活属于 `CCTRL_ERROR_REST_REQUIRED`，点亮ERR灯；不增加鸣叫、音乐、
  专用页面或三秒提示。
- 休息区域内屏蔽腕部接近限位提醒。关闭保护不关闭该区域判定和提醒屏蔽。

## 5. 校准页面

每轴显示原始计数/角度、offset、方向和补偿后角度。支持任意目标以及
`0°/-90°/90°/180°`；`+180°`和`-180°`合并为同一个语义。网页只形成候选，
点击保存后才上传完整六轴事务。
