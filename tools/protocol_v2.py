"""CCtrl-6DoF Protocol V2 reference encoder/decoder."""

from __future__ import annotations

import struct
from dataclasses import dataclass

BUS_SOF = 0xC65A
BUS_VERSION = 2
BUS_MAX_FRAME = 96
HOST_SOF = 0xA5
HOST_COMMAND = 0x0302
HOST_PAYLOAD_SIZE = 30
HOST_BODY_SIZE = 2 + HOST_PAYLOAD_SIZE


def crc8(data: bytes) -> int:
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value & 0xFF


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value & 0xFFFF


def build_bus(message_type: int, sequence: int, records: list[bytes] | None = None,
              flags: int = 0) -> bytes:
    records = records or []
    payload = b"".join(records)
    header = struct.pack("<H6B", BUS_SOF, BUS_VERSION, message_type, sequence,
                         len(records), len(payload), flags)
    frame = header + payload
    if len(frame) + 2 > BUS_MAX_FRAME:
        raise ValueError("bus frame exceeds 96 bytes")
    return frame + struct.pack("<H", crc16(frame))


def node_record(node_id: int, node_type: int, platform: int, data: bytes) -> bytes:
    return struct.pack("<4B", node_id, node_type, platform, len(data)) + data


def enum_record(node_id: int, node_type: int, platform: int,
                fw_major: int = 1, fw_minor: int = 0, capabilities: int = 1) -> bytes:
    return node_record(node_id, node_type, platform,
                       struct.pack("<BBH", fw_major, fw_minor, capabilities))


def encoder_record(node_id: int, platform: int, status: int, raw: int) -> bytes:
    return node_record(node_id, 1, platform, struct.pack("<BH", status, raw & 0xFFF))


def handle_record(node_id: int, platform: int, status: int, x: int, y: int,
                  magnetic_x: int, magnetic_y: int, magnetic_z: int,
                  buttons: int) -> bytes:
    return node_record(node_id, 2, platform,
                       struct.pack("<BHHhhhB", status, x, y, magnetic_x,
                                   magnetic_y, magnetic_z, buttons))


def parse_bus(frame: bytes) -> dict:
    if len(frame) < 10 or len(frame) > BUS_MAX_FRAME:
        raise ValueError("invalid bus frame size")
    sof, version, message_type, sequence, node_count, payload_len, flags = struct.unpack_from(
        "<H6B", frame)
    if (sof != BUS_SOF or version != BUS_VERSION or message_type not in (1, 2)
            or len(frame) != 10 + payload_len):
        raise ValueError("invalid bus header")
    if crc16(frame[:-2]) != struct.unpack_from("<H", frame, len(frame) - 2)[0]:
        raise ValueError("invalid bus CRC16")
    records = []
    cursor, end = 8, 8 + payload_len
    while cursor < end:
        if cursor + 4 > end:
            raise ValueError("truncated record header")
        node_id, node_type, platform, data_len = struct.unpack_from("<4B", frame, cursor)
        cursor += 4
        if cursor + data_len > end:
            raise ValueError("truncated record data")
        records.append({"node_id": node_id, "node_type": node_type,
                        "platform": platform, "data": frame[cursor:cursor + data_len]})
        cursor += data_len
    if len(records) != node_count:
        raise ValueError("node_count does not match records")
    return {"message_type": message_type, "sequence": sequence, "flags": flags,
            "records": records}


class BusStreamDecoder:
    """Incremental decoder that resynchronizes after noise or a bad frame."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[dict]:
        self.buffer.extend(data)
        decoded = []
        while True:
            start = self.buffer.find(b"\x5a\xc6")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b"\x5a" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 8:
                break
            total = 10 + self.buffer[6]
            if total > BUS_MAX_FRAME:
                del self.buffer[0]
                continue
            if len(self.buffer) < total:
                break
            candidate = bytes(self.buffer[:total])
            del self.buffer[:total]
            try:
                decoded.append(parse_bus(candidate))
            except ValueError:
                self.buffer[:0] = candidate[1:]
        return decoded


@dataclass
class HostPayload:
    system_status: int
    error_flags: int
    warning_flags: int
    node_valid_flags: int
    main_buttons: int
    handle_buttons: int
    encoder_status: tuple[int, ...]
    encoder_value: tuple[int, ...]
    joystick_x: int
    joystick_y: int
    trigger: int


def build_host(payload: HostPayload, sequence: int = 0) -> bytes:
    diagnostic = ((payload.system_status & 0x03)
                  | ((payload.error_flags & 0x01) << 2)
                  | ((payload.warning_flags & 0x0F) << 3))
    body = struct.pack("<4B6B6h3H2x", diagnostic,
                       payload.node_valid_flags,
                       payload.main_buttons, payload.handle_buttons,
                       *payload.encoder_status, *payload.encoder_value,
                       payload.joystick_x, payload.joystick_y,
                       payload.trigger)
    header4 = struct.pack("<BHB", HOST_SOF, len(body), sequence)
    frame = header4 + bytes([crc8(header4)]) + struct.pack("<H", HOST_COMMAND) + body
    return frame + struct.pack("<H", crc16(frame))


def parse_host(frame: bytes) -> HostPayload:
    if len(frame) < 9 or frame[0] != HOST_SOF:
        raise ValueError("invalid host SOF/size")
    payload_len = struct.unpack_from("<H", frame, 1)[0]
    if payload_len != HOST_PAYLOAD_SIZE or len(frame) != 9 + payload_len:
        raise ValueError("invalid host payload length")
    if crc8(frame[:4]) != frame[4] or crc16(frame[:-2]) != struct.unpack_from("<H", frame, len(frame) - 2)[0]:
        raise ValueError("invalid host CRC")
    if struct.unpack_from("<H", frame, 5)[0] != HOST_COMMAND:
        raise ValueError("unexpected command")
    values = struct.unpack_from("<4B6B6h3H2x", frame, 7)
    diagnostic = values[0]
    return HostPayload(diagnostic & 0x03, (diagnostic >> 2) & 0x01,
                       (diagnostic >> 3) & 0x0F, *values[1:4],
                       tuple(values[4:10]), tuple(values[10:16]),
                       *values[16:19])


@dataclass
class HostFrame:
    sequence: int
    payload: HostPayload


class HostStreamDecoder:
    """Incremental RM 0x0302 decoder that resynchronizes after bad frames."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, data: bytes) -> list[HostFrame]:
        self.buffer.extend(data)
        decoded = []
        while True:
            start = self.buffer.find(bytes([HOST_SOF]))
            if start < 0:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            payload_len = struct.unpack_from("<H", self.buffer, 1)[0]
            if payload_len != HOST_PAYLOAD_SIZE:
                del self.buffer[0]
                self.rejected += 1
                continue
            total = 9 + payload_len
            if len(self.buffer) < total:
                break
            candidate = bytes(self.buffer[:total])
            del self.buffer[:total]
            try:
                decoded.append(HostFrame(candidate[3], parse_host(candidate)))
            except ValueError:
                self.rejected += 1
                self.buffer[:0] = candidate[1:]
        return decoded
