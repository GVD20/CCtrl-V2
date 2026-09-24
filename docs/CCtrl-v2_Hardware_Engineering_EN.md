# CCtrl-V2 Hardware Engineering

[ZHCN](CCtrl-v2_Hardware_Engineering_ZHCN.md) | [EN](CCtrl-v2_Hardware_Engineering_EN.md)

This page lists the CCtrl-V2 electronics project and its four board designs.

## Main files and directories

- [JLCEDA Pro project](../hardware/electronics/jlc_eda_pro/CCtrl-v2.epro2): schematic and PCB project.
- [Electronics guide](../hardware/electronics/README.md): project format and board list.
- [Hardware index](../hardware/README.md): electronics and mechanical assets.

## Project contents

| Board | Project name | Role |
|---|---|---|
| Main board | `CC_ESP32main` | ESP32-S3 master and external interfaces |
| Handle module v2 | `Handle_node_v2` | Joystick, buttons, magnetic trigger |
| Encoder node with slip ring | `Encoder_node_v3` | Joint-angle sampling and daisy-chain relay |
| Slip-ring brush | `Encoder_brush` | Electrical connection across a rotating joint |

The electronics are provided as a native JLCEDA Pro project, from which manufacturing outputs are exported. The main board retains the original CCtrl design basis; the nodes and handle match the six-axis mechanism and V2 firmware.

## Related documents

- [User manual](CCtrl-v2_User_Manual_EN.md)
- [Embedded firmware](CCtrl-v2_Embedded_Firmware_EN.md)
- [Mechanical models](CCtrl-v2_Mechanical_Models_EN.md)
