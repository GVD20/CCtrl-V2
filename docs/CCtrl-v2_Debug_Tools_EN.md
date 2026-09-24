# CCtrl-V2 Debug Tools

[ZHCN](CCtrl-v2_Debug_Tools_ZHCN.md) | [EN](CCtrl-v2_Debug_Tools_EN.md)

This page lists the RS232 monitor, USB debugging page, and protocol reference implementation.

## Main files and directories

| Entry | Function |
|---|---|
| [rs232_channel_monitor.py](../tools/rs232_channel_monitor.py) | Parses RS232 `0x0302` frames, plots channels, exports CSV |
| [start_rs232_channel_monitor.bat](../tools/start_rs232_channel_monitor.bat) | Windows monitor launcher |
| [usb_robot_debug](../tools/usb_robot_debug) | Local Web Serial page, frame parsing, calibration, rest-pose controls |
| [protocol_v2.py](../tools/protocol_v2.py) | Python frame parser and CRC reference implementation |

## Launch commands

| Tool | Command | Interface |
|---|---|---|
| RS232 channel monitor | `python tools/rs232_channel_monitor.py` | RS232 adapter, 115200 8N1 |
| USB debug page | `python tools/usb_robot_debug/server.py` | `http://127.0.0.1:8768`; browser Web Serial at 2000000 baud |

The RS232 monitor reads 39-byte business frames and displays joints, validity, status, buttons, joystick, and trigger. Its plot window and CSV preserve raw protocol values. With `USB Debug → Start Debug` enabled on the device, the USB page shows raw-axis, XZY, ZXZ, and diagnostic snapshots at about 15 Hz and sends calibration and rest-pose configuration commands.

## Related documents

- [User manual](CCtrl-v2_User_Manual_EN.md)
- [Monitor guide](../tools/RS232_CHANNEL_MONITOR_ZHCN.md)
- [USB page guide](../tools/usb_robot_debug/README.md)
- [USB debug protocol](USB_DEBUG_ZHCN.md)
