# CCtrl-V2 Embedded Firmware

[ZHCN](CCtrl-v2_Embedded_Firmware_ZHCN.md) | [EN](CCtrl-v2_Embedded_Firmware_EN.md)

This page maps the CCtrl-V2 firmware environments, source directories, and runtime roles.

## Main files and directories

| Entry | Contents |
|---|---|
| [platformio.ini](../platformio.ini) | Three PlatformIO environments, dependencies, upload protocols |
| [src/master](../src/master) | ESP32-S3 master, calibration, wrist conversion, USB debugging |
| [src/node_avr](../src/node_avr) | ATmega328P Encoder node |
| [src/node_ch32v006](../src/node_ch32v006) | CH32V006 Encoder/Handle node |
| [src/shared](../src/shared) | Daisy-chain frames, CRC, shared protocol definitions |
| [src/ui_runtime](../src/ui_runtime) | OLED UI runtime |
| [include](../include), [lib](../lib) | UI configuration, business interfaces, local libraries |
| [test](../test) | Protocol, rest-pose, and wrist-conversion tests |

## Build commands

| Environment | Hardware and role | Command |
|---|---|---|
| `master_esp32s3` | ESP32-S3 master | `pio run -e master_esp32s3` |
| `encoder_node_atmega328p` | 8 MHz ATmega328P Encoder | `pio run -e encoder_node_atmega328p` |
| `encoder_node_ch32v006` | CH32V006 Encoder/Handle | `pio run -e encoder_node_ch32v006` |

The configured upload protocols are esptool for the master, USBasp for ATmega328P, and WCH-Link for CH32V006. The PlatformIO upload target is `pio run -e <environment> -t upload`.

## Runtime structure

The master discovers and polls nodes over a 250000 bps daisy-chain link at about 45 Hz. RS232 emits a 39-byte frame at 115200 bps and about 30 Hz. CH32V006 selects Encoder or Handle mode from the sensor detected at startup; ATmega328P implements the Encoder role. Six-axis calibration, wrist representation, and rest-pose settings are stored in ESP32 NVS.

## Related documents

- [User manual](CCtrl-v2_User_Manual_EN.md)
- [Daisy-Chain Protocol V2](PROTOCOL_V2.md)
- [RS232 serial protocol](SERIAL_PROTOCOL_ZHCN.md)
- [Encoder calibration](CALIBRATION_ZHCN.md)
