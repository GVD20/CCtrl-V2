import struct
import unittest

from tools.protocol_v2 import (
    BusStreamDecoder, HOST_BODY_SIZE, HostPayload, HostStreamDecoder, build_bus, build_host, crc16,
    encoder_record, enum_record, handle_record, parse_bus, parse_host,
)


class ProtocolV2Tests(unittest.TestCase):
    def test_rm_0302_data_is_thirty_bytes(self):
        self.assertEqual(HOST_BODY_SIZE, 32)

    def test_empty_poll_golden_vector(self):
        self.assertEqual(build_bus(2, 0x34).hex(), "5ac6020234000000c05d")

    def test_enumeration_and_platform_mix(self):
        records = [
            enum_record(0, 1, 1),
            enum_record(1, 2, 2),
            enum_record(2, 1, 2),
        ]
        parsed = parse_bus(build_bus(1, 7, records))
        slots = [r["node_id"] for r in parsed["records"] if r["node_type"] == 1]
        self.assertEqual(slots, [0, 2])

    def test_handle_position_does_not_change_encoder_order(self):
        for handle_at in range(7):
            records = []
            encoder_ids = []
            for node_id in range(7):
                if node_id == handle_at:
                    records.append(handle_record(node_id, 2, 3, 100, 200,
                                                 -1000, 2000, -3000, 7))
                else:
                    encoder_ids.append(node_id)
                    records.append(encoder_record(node_id, 1 + (node_id & 1), 3, node_id * 100))
            parsed = parse_bus(build_bus(2, 9, records))
            self.assertEqual([r["node_id"] for r in parsed["records"] if r["node_type"] == 1],
                             encoder_ids)

    def test_handle_record_keeps_signed_magnetic_axes(self):
        parsed = parse_bus(build_bus(2, 4, [handle_record(
            0, 2, 3, 123, 456, -32768, 0, 32767, 7)]))
        self.assertEqual(struct.unpack("<BHHhhhB", parsed["records"][0]["data"]),
                         (3, 123, 456, -32768, 0, 32767, 7))

    def test_one_to_six_encoder_degraded_masks(self):
        for count in range(1, 7):
            mask = sum(1 << i for i in range(count))
            self.assertEqual(mask == 0x3F, count == 6)

    def test_bad_crc_version_truncation_and_record_length(self):
        frame = bytearray(build_bus(2, 1, [encoder_record(0, 1, 3, 123)]))
        frame[-1] ^= 1
        with self.assertRaises(ValueError):
            parse_bus(bytes(frame))
        frame = bytearray(build_bus(2, 1))
        frame[2] = 3
        frame[-2:] = struct.pack("<H", crc16(frame[:-2]))
        with self.assertRaises(ValueError):
            parse_bus(bytes(frame))
        with self.assertRaises(ValueError):
            parse_bus(build_bus(3, 1))
        with self.assertRaises(ValueError):
            parse_bus(build_bus(2, 1)[:-1])
        frame = bytearray(build_bus(2, 1, [encoder_record(0, 1, 3, 123)]))
        frame[11] = 9
        frame[-2:] = struct.pack("<H", crc16(frame[:-2]))
        with self.assertRaises(ValueError):
            parse_bus(bytes(frame))

    def test_host_payload_offsets_endian_and_crc(self):
        payload = HostPayload(1, 0, 8, 0x7F, 5, 7,
                              (3, 3, 3, 3, 3, 3),
                              (1, 0x1234, 3, 4, 5, -1), 100, 200,
                              3686)
        frame = build_host(payload, 17)
        self.assertEqual(len(frame), 39)
        self.assertEqual(frame[1:3], b"\x1e\x00")
        self.assertEqual(frame[5:7], b"\x02\x03")
        self.assertEqual(frame[7:11], bytes([0x41, 0x7F, 5, 7]))
        self.assertEqual(frame[17:19], b"\x01\x00")
        self.assertEqual(frame[19:21], b"\x34\x12")
        self.assertEqual(frame[35:37], b"\x00\x00")
        self.assertEqual(parse_host(frame), payload)

    def test_rm_0302_golden_vector(self):
        payload = HostPayload(1, 0, 0, 0x7F, 3, 5,
                              (3, 3, 3, 3, 3, 3),
                              (0, 228, 683, -1, 2047, -2048),
                              2048, 2048, 4095)
        self.assertEqual(
            build_host(payload, 0).hex(),
            "a51e00007d0203017f03050303030303030000e400ab02ffff"
            "ff0700f800080008ff0f00005739")

    def test_host_encoder_values_are_signed_little_endian(self):
        payload = HostPayload(1, 0, 0, 0x3F, 0, 0,
                              (3, 3, 3, 3, 3, 3),
                              (-2048, -1, 0, 1, 1024, 2047), 0, 0, 0)
        frame = build_host(payload, 1)
        self.assertEqual(frame[17:29],
                         struct.pack("<6h", -2048, -1, 0, 1, 1024, 2047))
        self.assertEqual(parse_host(frame).encoder_value, payload.encoder_value)

    def test_serial_resynchronization(self):
        decoder = BusStreamDecoder()
        bad = bytearray(build_bus(2, 8))
        bad[-1] ^= 0x55
        good = build_bus(2, 9, [encoder_record(0, 2, 3, 456)])
        self.assertEqual(decoder.feed(b"noise\x5a"), [])
        decoded = decoder.feed(b"\xc6junk" + bytes(bad) + b"garbage" + good)
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0]["sequence"], 9)

    def test_host_serial_resynchronization_and_sequence(self):
        payload = HostPayload(1, 0, 0, 0x7F, 1, 2,
                              (3, 3, 3, 3, 3, 3),
                              (10, 20, 30, 40, 50, 60), 100, 200,
                              4095)
        bad = bytearray(build_host(payload, 6))
        bad[-1] ^= 0x80
        good = build_host(payload, 7)
        decoder = HostStreamDecoder()
        self.assertEqual(decoder.feed(b"noise" + bytes(bad) + good[:8]), [])
        frames = decoder.feed(good[8:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 7)
        self.assertEqual(frames[0].payload.encoder_value[5], 60)
        self.assertGreaterEqual(decoder.rejected, 1)


if __name__ == "__main__":
    unittest.main()
