# CCtrl-V2 User Manual

[ZHCN](CCtrl-v2_User_Manual_ZHCN.md) | [EN](CCtrl-v2_User_Manual_EN.md)

This manual describes the controller, firmware, communication links, output data, device settings, and tools. The [serial protocol](SERIAL_PROTOCOL_ZHCN.md) contains the complete byte layout; [Daisy-Chain Protocol V2](PROTOCOL_V2.md) defines the node link.

## 1. Project definition

CCtrl-V2 is a joint-space teleoperation controller for a six-degree-of-freedom serial robot arm with equal-length first and second links. Six magnetic encoders measure joints A1–A6. A handle node measures a joystick, S1–S3 buttons, and a magnetic trigger. The ESP32-S3 master handles node discovery and polling, calibration, the OLED menu, wrist-orientation representation, and output.

The controller outputs joint counts, handle inputs, and link status. The receiving robot controller implements its own geometry, joint zero positions, drive ratios, kinematics, and motor control.

| Component | Implementation | Data or role |
|---|---|---|
| Master | ESP32-S3 board | Calibration, menu, node polling, output |
| Joint nodes | Six AS5600 encoder nodes | Magnet status and raw angle, 4096 counts per revolution |
| Handle node | Handle module | Joystick, buttons, magnetic XYZ |
| External interface | RS232 | RoboMaster `0x0302` frame at about 30 Hz |
| Local debug interface | USB CDC | Manually enabled calibration and status snapshots |

## 2. Firmware and runtime structure

- [`src/master`](../src/master): ESP32-S3 polling, calibration, RS232, and USB debugging.
- [`src/node_avr`](../src/node_avr): ATmega328P encoder-node firmware.
- [`src/node_ch32v006`](../src/node_ch32v006): CH32V006 firmware, selecting Encoder or Handle mode from the detected sensor at startup.
- [`src/shared`](../src/shared): Daisy-Chain Protocol V2 and CRC code.
- [`src/ui_runtime`](../src/ui_runtime), [`include`](../include), [`lib`](../lib): OLED UI, shared interfaces, and local libraries.

The physical order of Encoder nodes defines A1–A6. The Handle has separate output fields and can occupy a position between Encoder nodes. The node-poll period is about 22.2 ms; the external-frame period is about 33.3 ms.

## 3. Communication links

### 3.1 External link

RS232 sends business frames from the ESP32-S3 to the receiving controller at 115200 bps, 8N1. A frame contains the `0xA5` header, length, sequence, CRC8, command ID `0x0302`, a 30-byte payload, and CRC16. Its total length is 39 bytes. USB debugging uses a separate channel.

### 3.2 Master and node link

The daisy-chain UART runs at 250000 bps. `ENUM_RESET (0x01)` assigns addresses in physical order; `POLL (0x02)` collects node records. An Encoder record contains status and raw angle. A Handle record contains joystick, magnetic XYZ, and buttons. See the [node protocol](PROTOCOL_V2.md) for framing, status bits, and LED behavior.

### 3.3 USB debug link

USB CDC is disabled at startup. `USB Debug → Start Debug` on the OLED menu enables the `CCtrl USB Debug` serial device, which the web page opens at 2000000 baud. The master sends raw-axis, XZY, ZXZ, diagnostic, and configuration snapshots at about 15 Hz. USB CDC remains enabled until reboot, while RS232 continues to carry business frames. The [USB debug protocol](USB_DEBUG_ZHCN.md) lists frame types and commands.

## 4. Unified output protocol (`0x0302`)

### 4.1 Frame structure

```text
SOF(0xA5) + data_length(2) + sequence(1) + CRC8(1)
+ cmd_id(0x0302, 2) + payload(30) + CRC16(2)
```

Multibyte integers use little-endian order. `data_length` is 30; the one-byte `sequence` increments modulo 256.

### 4.2 The 30-byte payload

| Offset | Length | Field | Type | Meaning |
|---:|---:|---|---|---|
| 0 | 1 | `diagnostic_flags` | `uint8` | System state, timeout, warnings |
| 1 | 1 | `node_valid_flags` | `uint8` | A1–A6 and Handle validity |
| 2 | 1 | `main_buttons` | `uint8` | KEY1–KEY4 single and double clicks |
| 3 | 1 | `handle_buttons` | `uint8` | S1–S4 states |
| 4–9 | 6 | `encoder_status[6]` | `uint8[6]` | Encoder sample states |
| 10–21 | 12 | `encoder_value[6]` | `int16[6]` | Calibrated joint counts |
| 22–27 | 6 | `joystick_x/y`, `trigger` | `uint16[3]` | Handle analog values, 0–4095 |
| 28–29 | 2 | `reserved` | `uint16` | Currently zero |

### 4.3 Status and validity

Bits 0–1 of `diagnostic_flags` encode `INIT`, `ACTIVE`, `DEGRADED`, or `DISCONNECTED`. Bit 2 marks link timeout; bits 3–6 mark CRC, format, magnet, and node-count warnings. Bits 0–5 of `node_valid_flags` correspond to A1–A6, and bit 6 to the Handle. During a sampling interruption, value fields retain their latest valid samples while validity and diagnostic fields report current state.

### 4.4 Joint counts

Each calibrated joint value ranges from `-2048` to `2047`, with 4096 counts per revolution. Degrees equal `encoder_value × 360 / 4096`. The circular wrap lies opposite the calibrated zero at `2047/-2048`. In `encoder_status`, bit 0 indicates I2C communication, bit 1 magnet detection, bits 2–3 weak or strong magnetic field, and bit 4 stale data. Status `0x03` indicates normal communication and magnet detection.

The [serial protocol](SERIAL_PROTOCOL_ZHCN.md) provides byte offsets, CRC algorithms, and a sample frame. Both CCtrl generations use command ID `0x0302`; CCtrl-V2 uses the payload layout above.

## 5. Output semantics

### 5.1 Joints and wrist

A1–A3 describe the main joints; A4–A6 describe the wrist. `Options → Wrist` selects `XZY`, which outputs calibrated A4–A6 directly, or `ZXZ`, which expresses the same wrist orientation in another Euler-angle convention. The conversion uses `Rx(-A4)·Rz(A5)·Ry(-A6) = Rz(α)·Rx(β)·Rz(γ)`. Near β=0° and β=180°, the firmware tracks a continuous solution branch. The selection is stored in NVS.

### 5.2 Buttons and analog inputs

The low four bits of `main_buttons` represent KEY1–KEY4 single-click events; the high four bits represent their double-click events. The firmware uses 20 ms debounce, a 300 ms double-click window, a 160 ms event pulse, and a 200 ms same-key refractory period. Handle bits 0–2 represent S1–S3. Bit 3 represents virtual S4, set at 80% trigger travel and cleared below 50%.

Joystick X/Y and trigger values range from 0 to 4095. The joystick center is 2048. The master maps raw ADC samples piecewise and reverses the Y output. The measured constants appear in [Handle joystick calibration](JOYSTICK_CALIBRATION_ZHCN.md).

### 5.3 Startup rest pose

`Options → RestGuard` stores the protection setting for the next startup. When enabled, RS232 emits the stored standard rest pose with `DEGRADED` status until all six joints enter the stored region for 500 ms. A1, A5, and A6 use ±30° entry and ±35° exit thresholds; A2–A4 use ±2° and ±3°. The USB page stores the current healthy, fresh raw samples as the rest pose.

## 6. Device settings and calibration

The OLED `Calibration → Encoder_calibration` menu contains `Zero All 6`, A1–A6 `Reverse`, `Save`, and `Reload`. `Zero All 6` captures the current raw samples and assigns 0° to A1 and A4–A6, +20° (about 228 counts) to A2, and +60° (about 683 counts) to A3. The operation writes the complete calibration set to NVS when all six nodes are present, I2C communication works, and magnets are detected.

Calibration contains six offsets and directions in a versioned NVS blob with CRC16. The USB page displays raw counts, offsets, directions, and calibrated angles. It offers 0°, ±90°, 180°, and custom targets, saving all six axes as one revisioned, CRC-protected transaction. The calculation is documented in [Encoder calibration](CALIBRATION_ZHCN.md).

## 7. Host tools

- [RS232 channel monitor](../tools/RS232_CHANNEL_MONITOR_ZHCN.md): `python tools/rs232_channel_monitor.py`; displays decoded fields and plots and exports raw-value CSV data.
- [USB Web Serial debugger](../tools/usb_robot_debug/README.md): `python tools/usb_robot_debug/server.py`; serves the local page at `http://127.0.0.1:8768` for raw axes, XZY/ZXZ, diagnostics, calibration, and rest pose.
- [Protocol parser](../tools/protocol_v2.py): Python reference parser and CRC implementation for RS232 frames.

The [debug tools index](CCtrl-v2_Debug_Tools_EN.md) lists these entries and their functions.

## 8. Build and upload

[`platformio.ini`](../platformio.ini) defines three environments:

| Environment | Hardware | Programmer | Build command |
|---|---|---|---|
| `master_esp32s3` | ESP32-S3 master | esptool | `pio run -e master_esp32s3` |
| `encoder_node_atmega328p` | 8 MHz ATmega328P Encoder | USBasp | `pio run -e encoder_node_atmega328p` |
| `encoder_node_ch32v006` | CH32V006 node | WCH-Link | `pio run -e encoder_node_ch32v006` |

The PlatformIO upload target is `pio run -e <environment> -t upload`. The [firmware index](CCtrl-v2_Embedded_Firmware_EN.md) maps files and runtime responsibilities.

## 9. Project assets and verification

- [Hardware engineering](CCtrl-v2_Hardware_Engineering_EN.md): the four-board JLCEDA Pro project.
- [Mechanical models](CCtrl-v2_Mechanical_Models_EN.md): assembly STEP, improved replacement-parts STEP, and transparent cover image.
- [Node protocol](PROTOCOL_V2.md): enumeration, polling, records, status, and LEDs.
- [External serial protocol](SERIAL_PROTOCOL_ZHCN.md): 39-byte frame, 30-byte payload, and byte definitions.

All three PlatformIO firmware environments have built successfully. The 12 protocol tests, C++ rest-pose and wrist-conversion tests, and USB frame-parser self-test have passed.
