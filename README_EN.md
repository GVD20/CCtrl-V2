# CCtrl-v2

![CCtrl-v2 render](hardware/mechanical/renders/CCtrl-v2-cover.png)

[简体中文](README.md) · [User manual (Chinese)](docs/CCtrl-v2_User_Manual_ZHCN.md) · [RS232 protocol (Chinese)](docs/SERIAL_PROTOCOL_ZHCN.md)

CCtrl-v2 is a joint-space teleoperation controller for a six-degree-of-freedom serial robot arm whose first two links have equal length. Six magnetic encoders measure joints A1–A6; a handle node supplies a joystick, buttons, and a magnetic trigger. An ESP32-S3 master handles node polling, calibration, the OLED menu, wrist-orientation representation, and output. The robot-side controller must apply its own geometry, zero positions, direction conventions, and safety limits.

## Relationship to CCtrl

CCtrl-v2 retains the main-board design, RoboMaster `0x0302` frame envelope, and MIT license from [GVD20/CCtrl](https://github.com/GVD20/CCtrl). Its mechanism, node chain, and payload have been redesigned:

| Area | Original CCtrl | CCtrl-v2 |
|---|---|---|
| Sensing | Three encoders and an IMU | Six joint encoders and a new handle |
| Node link | Original node protocol | Daisy-Chain Protocol V2 |
| Output | Fused position, orientation, and wheel data | Joint counts, handle inputs, and status in a [30-byte V2 payload](docs/SERIAL_PROTOCOL_ZHCN.md) |
| Host workflow | WebXR | RS232 channel monitor and local USB debugger |

Both versions use command ID `0x0302`; receiving software selects the corresponding payload format.

## Documentation and tools

| Area | File |
|---|---|
| Operation, calibration, troubleshooting | [Detailed user manual (Chinese)](docs/CCtrl-v2_User_Manual_ZHCN.md) |
| Node-chain protocol | [Protocol V2](docs/PROTOCOL_V2.md) |
| External RS232 protocol | [Serial protocol (Chinese)](docs/SERIAL_PROTOCOL_ZHCN.md) |
| Electronics and CAD | [Hardware guide](hardware/README.md) |
| RS232 channel monitor | [Monitor guide](tools/RS232_CHANNEL_MONITOR_ZHCN.md) |
| USB Web Serial calibration/debugging | [USB debugger guide](tools/usb_robot_debug/README.md) |

Build the `master_esp32s3`, `encoder_node_atmega328p`, and `encoder_node_ch32v006` PlatformIO environments with `pio run`. CH32V006 firmware selects encoder or handle mode according to the detected sensor; the ATmega328P environment is for encoder nodes.

## Setup flow

1. Open the four-board project in JLCEDA Pro and review the assembly and replacement parts in CAD.
2. Build and upload the master and node firmware. Connect six encoder nodes in joint order, along with the handle node.
3. Check node discovery, magnet status, and axis validity on the OLED `Monitor` and `Debug` screens.
4. Place the mechanism in its reference pose, run `Calibration → Encoder_calibration`, and verify each axis direction.
5. Inspect the 39-byte RS232 output with the [channel monitor](tools/RS232_CHANNEL_MONITOR_ZHCN.md). Use the [USB debugger](tools/usb_robot_debug/README.md) for individual-axis targets and the rest pose.
6. Implement joint mapping and motion handling on the robot side using the [CCtrl-v2 serial payload](docs/SERIAL_PROTOCOL_ZHCN.md).

The [user manual](docs/CCtrl-v2_User_Manual_ZHCN.md) covers assembly, calibration, operation, and troubleshooting in detail.

The electronics are supplied as a four-board **JLCEDA Pro** project, from which manufacturing outputs can be exported. Mechanical assets include a full-assembly STEP, an improved replacement-parts STEP, and a transparent cover render. Check the replacement-part correspondence and fits in CAD during assembly.

The project uses the [MIT license](LICENSE). The UI adapts work from [RQNG/WouoUI](https://github.com/RQNG/WouoUI).
