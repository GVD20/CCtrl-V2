<h1 align="center">CCtrl-V2</h1>

<p align="center">
  <strong>Joint-space teleoperation controller for six-axis robot arms</strong><br>
  Equal-length first two links · XZY/ZXZ wrist-orientation output
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
  <img alt="CCtrl-V2 render" src="hardware/mechanical/renders/CCtrl-v2-cover.png">
</p>

CCtrl-V2 measures joints A1–A6 with six magnetic encoders and receives joystick, button, and magnetic-trigger input from a Handle node. An ESP32-S3 master handles node polling, calibration, the OLED menu, wrist-orientation representation, and RS232 output. This repository contains firmware, a four-board JLCEDA Pro project, mechanical models, documentation, and debugging tools.

<h5>▌Quick Links</h5>

| Module | ZHCN | EN |
|---|---|---|
| User Manual | [CCtrl-v2_User_Manual_ZHCN](docs/CCtrl-v2_User_Manual_ZHCN.md) | [CCtrl-v2_User_Manual_EN](docs/CCtrl-v2_User_Manual_EN.md) |
| Embedded Firmware | [CCtrl-v2_Embedded_Firmware_ZHCN](docs/CCtrl-v2_Embedded_Firmware_ZHCN.md) | [CCtrl-v2_Embedded_Firmware_EN](docs/CCtrl-v2_Embedded_Firmware_EN.md) |
| Hardware Engineering | [CCtrl-v2_Hardware_Engineering_ZHCN](docs/CCtrl-v2_Hardware_Engineering_ZHCN.md) | [CCtrl-v2_Hardware_Engineering_EN](docs/CCtrl-v2_Hardware_Engineering_EN.md) |
| Mechanical Models | [CCtrl-v2_Mechanical_Models_ZHCN](docs/CCtrl-v2_Mechanical_Models_ZHCN.md) | [CCtrl-v2_Mechanical_Models_EN](docs/CCtrl-v2_Mechanical_Models_EN.md) |
| Debug Tools | [CCtrl-v2_Debug_Tools_ZHCN](docs/CCtrl-v2_Debug_Tools_ZHCN.md) | [CCtrl-v2_Debug_Tools_EN](docs/CCtrl-v2_Debug_Tools_EN.md) |

Technical references: [Node protocol](docs/PROTOCOL_V2.md) · [RS232 protocol](docs/SERIAL_PROTOCOL_ZHCN.md) · [Encoder calibration](docs/CALIBRATION_ZHCN.md) · [USB debug protocol](docs/USB_DEBUG_ZHCN.md)

<h5>▌Highlights</h5>

- An ESP32-S3 master connects to six Encoder nodes and one Handle node through a 250000 bps daisy-chain UART. Node firmware targets ATmega328P and CH32V006.
- Six AS5600 encoders map to A1–A6 in physical chain order. The Handle provides a joystick, S1–S3 buttons, and a magnetic trigger that generates S4.
- RS232 emits fixed 39-byte RoboMaster `0x0302` frames at about 30 Hz. USB CDC provides manually enabled local debugging and calibration.
- The OLED menu provides six-axis calibration, startup rest-pose protection, and XZY/ZXZ wrist-output selection.
- The controller emits joint counts and status; the receiving robot controller maps these to its geometry, zero positions, directions, and motion control.

<h5>▌Relationship to CCtrl</h5>

CCtrl-V2 retains the MIT license, main-board design, and RoboMaster `0x0302` frame envelope from [GVD20/CCtrl](https://github.com/GVD20/CCtrl). Its mechanism, node link, and payload have been redesigned.

| Area | Original CCtrl | CCtrl-V2 |
|---|---|---|
| Mechanism and sensing | Spatial interaction with three encoders and an IMU | Six joint encoders for an arm with equal-length first and second links |
| Node link | Original node protocol | Daisy-Chain Protocol V2 with Encoder and Handle nodes |
| External data | Fused position, orientation, and wheel data | Joint counts, Handle inputs, and status in a [30-byte V2 payload](docs/SERIAL_PROTOCOL_ZHCN.md) |
| Host workflow | WebXR | RS232 channel monitor and local USB calibration/debugging page |

Both versions use command ID `0x0302`; receiving software selects the corresponding payload format.

<h5>▌Repository Structure</h5>

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

<h5>▌Firmware Build</h5>

```bash
pio run -e master_esp32s3
pio run -e encoder_node_atmega328p
pio run -e encoder_node_ch32v006
```

CH32V006 selects Encoder or Handle mode according to the sensor detected at startup; ATmega328P implements the Encoder role. The [firmware guide](docs/CCtrl-v2_Embedded_Firmware_EN.md) lists the configured upload protocols.

<h5>▌Host and Debug Tools</h5>

```bash
python tools/rs232_channel_monitor.py
python tools/usb_robot_debug/server.py
```

- [RS232 channel monitor](tools/RS232_CHANNEL_MONITOR_ZHCN.md): displays joints, status, buttons, and Handle data from 39-byte frames; plots channels and exports CSV.
- [USB debugger](tools/usb_robot_debug/README.md): displays raw samples, XZY/ZXZ, diagnostics, and calibration settings. RS232 continues to carry business frames.

<h5>▌Hardware Assets</h5>

The [JLCEDA Pro project](hardware/electronics/jlc_eda_pro/CCtrl-v2.epro2) contains the main board, Handle module v2, Encoder node with slip ring, and slip-ring brush. Mechanical assets include the [full-assembly STEP](hardware/mechanical/cad/CCtrl-v2-assembly.step), [improved replacement-parts STEP](hardware/mechanical/cad/CCtrl-v2-replacement-parts.step), and transparent cover render.

<h5>▌PlatformIO Library Dependencies</h5>

- `encoder_node_atmega328p`: `robtillaart/AS5600`
- `master_esp32s3`: `frankboesing/FastCRC`, `olikraus/U8g2`
- Local libraries: `lib/WouoUiLiteGeneralBridge`, `lib/WouoUiLiteGeneralOfficial`

<h5>▌Verification</h5>

All three PlatformIO firmware environments have built successfully. Protocol parsing, rest-pose, wrist conversion, and USB frame parsing have passed source-level tests.

<h5>▌UI Framework Credit</h5>

The UI implementation adapts work from [RQNG/WouoUI](https://github.com/RQNG/WouoUI).

<h5>▌License</h5>

The project uses the MIT license. See [LICENSE](LICENSE).
