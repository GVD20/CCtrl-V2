# CCtrl-v2 控制器串口协议说明

本文档描述 CCtrl-v2 ESP32-S3 主机向外部控制器输出的 RS232 二进制协议。

## 1. 串口参数与输出周期

| 项目 | 当前值 |
|---|---:|
| 物理接口 | RS232 |
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无 |
| 输出周期 | 约30 Hz（33334 us，不超过 RM 30 Hz上限） |
| 数据方向 | ESP32-S3 主机向外部控制器单向输出 |

主机每个输出周期发送一帧。没有新的有效节点采样时，数值字段继续发送最近一次有效值；有效位和诊断字段反映当前状态。

启用上电休息位姿保护时，设备在确认物理姿态回到休息区域并连续稳定500 ms前，
六轴字段发送保存的标准休息位姿，系统状态为 `DEGRADED`。该功能不改变39字节
RS232帧格式；手动启动的USB调试通道同时提供的XZY和ZXZ双份数据不会加入
RS232帧。一般工作输出始终只使用RS232。

## 2. 完整帧

所有多字节整数均按小端顺序传输。

| 帧偏移 | 字段 | 类型 | 当前值或含义 |
|---:|---|---|---|
| 0 | `sof` | `uint8` | 固定 `0xA5` |
| 1–2 | `payload_len` | `uint16` | 固定 `30`，线上字节为 `1E 00` |
| 3 | `sequence` | `uint8` | 每帧加 1，`255` 后回到 `0` |
| 4 | `header_crc8` | `uint8` | 覆盖偏移 0–3 |
| 5–6 | `cmd_id` | `uint16` | 固定 `0x0302`，线上字节为 `02 03` |
| 7–36 | `payload` | 30 bytes | CCtrl 当前业务数据，符合 RM `0x0302` 固定长度 |
| 37–38 | `frame_crc16` | `uint16` | 覆盖偏移 0–36 |

完整帧固定为39字节。RoboMaster 帧头中的 `payload_len` 只表示 `data` 长度，
不包含2字节 `cmd_id`；因此 `0x0302` 的 data 固定为30字节。

## 3. Payload

下表的偏移以30字节 payload 的第一个字节为0；换算为完整帧偏移时加7。

| Payload 偏移 | 字段 | 类型 | 可用值 |
|---:|---|---|---|
| 0 | `diagnostic_flags` | `uint8` | 系统状态、错误和警告的组合位 |
| 1 | `node_valid_flags` | `uint8` | 六路 Encoder 与 Handle 的当前有效位 |
| 2 | `main_buttons` | `uint8` | ESP32 主机快捷键的单击/双击事件 |
| 3 | `handle_buttons` | `uint8` | Handle 的 S1–S4 |
| 4–9 | `encoder_status[6]` | `uint8[6]` | A1–A6 节点状态 |
| 10–21 | `encoder_value[6]` | `int16[6]` | A1–A6 校准后的有符号位置计数 |
| 22–23 | `joystick_x` | `uint16` | Handle 摇杆 X 行程 |
| 24–25 | `joystick_y` | `uint16` | Handle 摇杆 Y 行程，方向已在主机中反转 |
| 26–27 | `trigger` | `uint16` | Handle 扳机行程 |
| 28–29 | `reserved` | `uint16` | 保留，当前发送 `0`，接收端忽略 |

### 3.1 diagnostic_flags

| 位 | 名称 | 含义 |
|---:|---|---|
| bit0–1 | `system_status` | `0 INIT`、`1 ACTIVE`、`2 DEGRADED`、`3 DISCONNECTED` |
| bit2 | `timeout_error` | `1` 表示 500 ms 内没有收到协议与 CRC 均正确的有效菊花链帧 |
| bit3 | `crc_warning` | `1` 表示最近 2 秒检测过菊花链 CRC 错误 |
| bit4 | `format_warning` | `1` 表示最近 2 秒检测过不符合协议的数据格式 |
| bit5 | `magnet_warning` | `1` 表示最近 2 秒检测过 Encoder 磁场异常或未检测到磁铁 |
| bit6 | `node_count_warning` | `1` 表示最近 2 秒 POLL 节点数量与上次成功枚举不一致 |
| bit7 | 保留 | 当前为 `0` |

字段拆分方式：

```text
system_status  = diagnostic_flags & 0x03
timeout_error  = (diagnostic_flags >> 2) & 0x01
warning_flags  = (diagnostic_flags >> 3) & 0x0F
```

### 3.2 node_valid_flags

| 位 | 对应数据 |
|---:|---|
| bit0–bit5 | A1–A6 Encoder |
| bit6 | Handle |
| bit7 | 保留 |

某个 Encoder 位为 `1` 时，对应的 `encoder_status[n]` 和 `encoder_value[n]` 是当前有效数据。位为 `0` 时，`encoder_value[n]` 可能仍保留最近一次有效值；从未收到有效值时为 `0`。

Handle bit6 为 `1` 时，`handle_buttons`、`joystick_x`、`joystick_y` 和 `trigger` 是当前有效数据。Handle 缺失或尚未获得有效采样时该位为 `0`。

### 3.3 按键

`main_buttons`：

| 位 | 含义 |
|---:|---|
| bit0–bit3 | KEY1–KEY4单击事件 |
| bit4–bit7 | KEY1–KEY4双击事件，分别对外表示KEY5–KEY8 |

KEY1–KEY4先经过20ms软件去抖。非磁贴菜单模式下，第一次有效按下开启300ms双击窗口；窗口超时才发送对应低位单击，窗口内第二次有效按下则立即发送对应高位双击且不会发送低位。事件位持续160ms，判定后同一物理按键进入200ms不应期，期间不响应也不发声。磁贴菜单模式保持原电平行为，只直接使用bit0–bit3。

主机上的三个菜单导航键不进入 `main_buttons`。

`handle_buttons`：

| 位 | 含义 |
|---:|---|
| bit0 | S1；按下为 `1` |
| bit1 | S2；按下为 `1` |
| bit2 | S3，摇杆按键；按下为 `1` |
| bit3 | S4，扳机数字状态；按下为 `1` |
| bit4–bit7 | 保留 |

S4 在扳机行程达到 80% 时置位，并保持到行程低于 50% 时清除。

### 3.4 Encoder 状态

每个 `encoder_status[n]` 使用以下位：

| 位 | 含义 |
|---:|---|
| bit0 | AS5600 I2C 通讯正常 |
| bit1 | 检测到磁铁 |
| bit2 | 磁场过弱 |
| bit3 | 磁场过强 |
| bit4 | 数据过期 |
| bit5–bit7 | 保留 |

当前正常且检测到磁铁时数值为 `0x03`。

### 3.5 Encoder 数值口径

`encoder_value[0]` 到 `encoder_value[5]` 对应 A1 到 A6。每路均为主机校准后的 `int16` 有符号计数，范围为 `-2048` 到 `2047`，一圈为 4096 计数：

```text
角度（度） = encoder_value * 360 / 4096
角度（弧度） = encoder_value * 2π / 4096
```

方向已经使用主机保存的逐轴 `direction` 配置处理。执行主机的六轴置零后，A1、A4、A5、A6 的当前位置输出为 0；A2 的当前位置输出为约 `+20°`（228 计数），A3 的当前位置输出为约 `+60°`（683 计数）。环形折返点位于 `-2048/2047`。

主机菜单 `Options -> Wrist` 为 `XZY` 时，A4、A5、A6 分别直接输出校准后的 X、Z、Y 角。设置为 `ZXZ` 时，转换输入与配套网页一致，先按 `X=-A4, Z=A5, Y=-A6` 组成右手系内禀主动旋转
`R = Rx(-A4) · Rz(A5) · Ry(-A6)`，再将同一姿态转换为
`R = Rz(α) · Rx(β) · Rz(γ)`，并依次通过 A4、A5、A6 输出。ZXZ 模式只改变 A4–A6 的对外输出，不改变节点采集值或保存的逐轴校准值。该选项保存在 ESP32 NVS 中。

ZXZ 的 `β=0°` 和 `β=180°` 附近为欧拉角奇异区。当前固件在距奇异点 3° 内进入连续跟随，保持可观测的 `α+γ` 或 `α−γ`，并将变化平均分配给两个 Z 轴；距奇异点达到 5°后退出。退出时按每个30 Hz输出帧、每轴最多9°逐步恢复精确解。常规区从两个等价 ZXZ 解中选择最接近上一帧的分支。

### 3.6 Handle 模拟量

| 字段 | 范围 | 中心或端点 |
|---|---:|---|
| `joystick_x` | 0–4095 | 中心 2048，已包含中央死区 |
| `joystick_y` | 0–4095 | 中心 2048，已包含中央死区，输出方向已反转 |
| `trigger` | 0–4095 | 0 为完全松开，4095 为完全按下，端点包含死区 |

百分比换算为：`value * 100 / 4095`。

## 4. CRC 算法

Header CRC8 的初值为 `0xFF`，反射多项式为 `0x8C`。Frame CRC16 的初值为 `0xFFFF`，反射多项式为 `0x8408`。CRC16 结果以小端顺序放在帧尾。

```c
uint8_t cctrl_crc8(const uint8_t *data, size_t length) {
    uint8_t value = 0xFF;
    while (length--) {
        value ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            value = (value >> 1) ^ ((value & 1) ? 0x8C : 0);
    }
    return value;
}

uint16_t cctrl_crc16(const uint8_t *data, size_t length) {
    uint16_t value = 0xFFFF;
    while (length--) {
        value ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            value = (value >> 1) ^ ((value & 1) ? 0x8408 : 0);
    }
    return value;
}
```

## 5. 控制器接收流程

1. 在接收字节流中定位 `0xA5`。
2. 收齐前5字节，读取小端 `payload_len`；RM `0x0302` 只接受 `30`。
3. 校验偏移 0–3 的 CRC8 是否等于偏移 4。
4. 按固定长度收齐39字节。
5. 检查偏移5–6的小端命令号是否为 `0x0302`。
6. 校验偏移0–36的 CRC16 是否等于偏移37–38的小端值。
7. CRC 与格式均正确后，按第 3 节解析 payload。
8. 使用 `sequence` 的模 256 差值统计帧序号连续性。
9. 校验失败时丢弃当前候选帧，并从后续字节重新寻找 `0xA5`。

## 6. 示例帧

以下示例的 `sequence=0`，系统状态为 ACTIVE，A1–A6 全部正常，六轴值依次为 `0, 228, 683, -1, 2047, -2048`，摇杆居中，扳机为 4095：

```text
A5 1E 00 00 7D 02 03 01 7F 03 05 03 03 03 03 03 03 00 00 E4 00 AB 02 FF FF FF 07 00 F8 00 08 00 08 FF 0F 00 00 57 39
```
