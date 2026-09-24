# 六路 Encoder 校准

校准数据以一个带版本与 CRC16 的 NVS Blob 保存，其中包含六路 offset 和六路
direction。任何无效 Blob 都会整体拒绝，不会进行部分载入。

OLED 的 `Calibration -> Encoder_calibration` 提供：

- `Zero All 6`：仅当六路 Encoder 均存在、I2C 正常且检测到磁铁时执行。磁场弱、
  磁场强和 stale 状态不阻止置零。操作一次采集六路 raw，并原子写入 NVS；任一路
  不满足基本条件则原有校准保持不变。
- `A1 Reverse` 到 `A6 Reverse`：切换对应方向。
- `Save`：保存方向和 offset。
- `Reload`：从 NVS 重新载入完整 Blob。

计算先在模 4096 下完成：`delta = direction * (raw - offset)`，再将结果转换为
有符号 `int16`。置零位置输出 0，范围为 -2048–2047；经过零位时连续穿过正负数，
环形折返点移动到零位正对面的 -2048/2047 边界。

`Zero All 6` 使用以下目标值：

| 轴 | 当前姿态校准后的值 |
|---|---:|
| A1 | 0° |
| A2 | +20° |
| A3 | +60° |
| A4 | 0° |
| A5 | 0° |
| A6 | 0° |

目标角度会换算为最接近的 4096 计数，并考虑该轴当前的方向设置。A2/A3 即使设为
反向，执行 `Zero All 6` 后仍分别得到正的 +20°/+60°。

Web Serial USB调试器也可编辑任意单轴目标、offset和方向。网页修改仅作为候选，
点击保存后才将完整六轴配置以一个带revision与CRC的事务写入ESP32；写入失败时
继续使用原配置。`+180°`和`-180°`合并为同一个 `180°` 语义。协议和休息位姿说明
见 [USB_DEBUG_ZHCN.md](USB_DEBUG_ZHCN.md)。
