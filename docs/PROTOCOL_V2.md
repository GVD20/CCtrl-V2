# Daisy-Chain Protocol V2

## 总线

总线速率 250000 bps，ESP32-S3 以 45 Hz 轮询。所有多字节字段均为小端。
主机向 RS232 上位机的 RoboMaster `0x0302` 输出独立固定为30 Hz。USB只承载手动
启动的调试协议，不承载一般业务帧。

`main_buttons`低4位表示KEY1–KEY4单击，高4位表示相同物理键的双击
（对外命名KEY5–KEY8）。快捷键使用20ms去抖、300ms双击窗口、160ms事件脉冲和
200ms同键不应期；双击不会同时产生单击位。三个菜单导航键不进入主机帧。

帧头为 8 字节：`sof(0xC65A), version(2), message_type, sequence,
node_count, payload_len, flags`，随后为 payload 和 RoboMaster CRC16。CRC16 覆盖
帧头与 payload，完整帧不超过 96 字节。

消息类型只有：

- `0x01 ENUM_RESET`：清除地址并按物理链路顺序枚举。
- `0x02 POLL`：逐节点追加采样记录。

节点完整收帧并校验 CRC，CRC 错误时丢弃该帧；CRC 正确、消息类型未识别的帧
保持原样传给下游。链路超时由主控判断。

节点记录头为 `node_id, node_type, platform, data_len`。`node_type` 中 Encoder=1、
Handle=2；`platform` 中 ATmega328P=1、CH32V006=2。

- 枚举数据：`fw_major, fw_minor, capabilities(uint16)`。
- Encoder：`status_flags, raw_angle(uint16)`，数据长度 3。
- Handle：`status_flags, joystick_x(uint16), joystick_y(uint16),
  magnetic_x(int16), magnetic_y(int16), magnetic_z(int16), buttons(uint8)`，数据长度 12。

Encoder 状态位：bit0 I2C 正常、bit1 检测到磁场、bit2 磁场过弱、bit3 磁场
过强、bit4 数据过期。Encoder slot 只按枚举结果中 Encoder 出现的次序生成，Handle
插入任意位置均不改变 Encoder 映射。

Handle 状态位：bit0 I2C 正常、bit1 TMAG 数据有效、bit2 数据过期。节点记录中的
`buttons` bit0/1/2 分别为 S1/S2/S3。ESP32 根据 TMAG XYZ 轨迹计算 0–4095 扳机
行程。行程达到 80%（计数不低于 3276）时将 `handle_buttons` bit3 置位为虚拟
S4，并保持到行程低于 50%（计数低于 2048）再清除。

## 主机诊断

主机内部错误位包括 `bit0 = 500 ms 内没有收到协议和 CRC 都正确的有效帧`，以及 `bit1 = 上电休息位姿保护尚未解除`。对外 RS232 的 `diagnostic_flags.bit2` 只编码链路超时；休息位姿保护通过 `DEGRADED` 状态表示，详细标志可从 USB 调试通道读取。

警告在最后一次发现对应问题后保持 2 秒：

| 位 | 含义 |
|---:|---|
| bit0 | CRC 校验失败 |
| bit1 | 帧头、消息类型或节点记录格式不符合协议 |
| bit2 | Encoder 磁场弱、强或未检测到磁铁 |
| bit3 | POLL 节点数量与最近一次成功枚举不一致 |

节点数量警告在 POLL 与最近一次成功枚举的节点数量不一致时触发。主机根据
连续有效 POLL 的 sequence 差值累计丢失帧数，重复或迟到的旧帧不重复计数。

## 上位机

保留 RoboMaster 外层：`0xA5 + payload_len + seq + header_crc8 + cmd_id +
payload + crc16`。命令为 `0x0302`，`payload_len` 按 RM 定义只计算 data，固定
为30字节；完整帧为39字节：

| 偏移 | 字段 | 类型 |
|---:|---|---|
| 0 | diagnostic_flags | uint8 |
| 1 | node_valid_flags | uint8 |
| 2 | ESP32 四按键 | uint8 |
| 3 | Handle 四按键 | uint8 |
| 4 | 六路 Encoder 状态 | uint8[6] |
| 10 | 六路校准后有符号计数 | int16[6] |
| 22 | joystick X | uint16 |
| 24 | joystick Y | uint16 |
| 26 | 扳机行程 | uint16 |
| 28 | 保留，当前为0 | uint16 |

`diagnostic_flags` 的 bit0–1 为 system_status，bit2 为 timeout error，bit3–6
依次为 CRC、格式、磁场、节点数量警告，bit7 保留。命令号 `0x0302` 标识
RoboMaster 外层帧；CCtrl-v2 使用本节的 30 字节 payload 布局。system_status：
0 INIT、1 ACTIVE、2 DEGRADED、3 DISCONNECTED。`node_valid_flags` bit0–5
对应六路 Encoder，bit6 对应 Handle。无效 Encoder 保留最后值并清除有效位；
Handle 采样仅在记录存在、状态有效且 XYZ 不是全零时更新。暂时缺少新有效采样、
链路超时或收到坏帧时，模拟量与按键继续输出最后一次有效值，同时通过有效位和
诊断字段反映当前链路状态，不再注入临时零值。固定载荷长度为30字节，末尾2字节
为保留字段且当前发送0。参考实现
位于 `tools/protocol_v2.py`。

六路 Encoder 的置零位置为有符号 0，输出范围为 -2048–2047。经过置零位置时数值
连续穿过 0；模 4096 的折返点位于零位正对面的 -2048/2047 边界。

`Options -> Wrist` 可在 XZY 与 ZXZ 末端轴系之间切换并保存到 NVS。XZY 模式直接
输出 A4/A5/A6 的 X/Z/Y 校准角；ZXZ 模式按右手系内禀主动旋转
`Rx(-A4)·Rz(A5)·Ry(-A6) = Rz(alpha)·Rx(beta)·Rz(gamma)` 转换最终三个
输出值，采用配套网页相同的 `X=-A4, Z=A5, Y=-A6` 方向约定。
`beta=0°/180°` 奇异点 3° 内进入连续跟随，达到 5°后退出；退出时每个30 Hz
输出帧每轴最多恢复9°。常规区选择与上一帧最接近的等价 ZXZ 分支。

Joystick X/Y 由 ESP32 对 Handle 原始 ADC 值进行分段线性校准，统一输出 0–4095，
中心为 2048。X 使用原始范围 805/2010/3226，Y 使用 857/1985/3150；两个轴在
中心两侧各保留 50 个原始 ADC 码的死区。Y 轴在校准后反向输出，X 轴方向不变。

S1–S4 任一按键从松开变为按下时，ESP32 主机播放一次按键提示音；持续按住不会
重复触发。S4 在扳机降到 50% 以下而解除时，另播放一次较低音调的退出提示音。

## 节点 LED

Encoder 的 STAT 和 ERR 上电同时闪两次；识别为 Handle 时，ERR 和 STAT 上电各闪
两次且交替出现，用于直接确认自动识别结果。枚举后，STAT 每 5 秒按用户可见编号
闪烁 `node_id+1` 次。Handle 任一按键按下时 STAT 立即点亮，保持到松开且每次至少
点亮 200 ms；按键显示优先于其他灯效。ERR 闪烁次数：1=未检测到磁铁、2=磁场弱、
3=磁场强、4=最近 2 秒发生 CRC 丢帧、5=I2C 设备不可用。
