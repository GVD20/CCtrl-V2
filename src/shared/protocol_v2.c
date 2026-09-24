#include "protocol_v2.h"
#include "crc.h"

#include <string.h>

uint16_t cctrl_read_u16_le(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void cctrl_write_u16_le(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

void cctrl_bus_init(uint8_t *frame, uint8_t message_type, uint8_t sequence) {
  memset(frame, 0, CCTRL_BUS_HEADER_SIZE + CCTRL_BUS_CRC_SIZE);
  cctrl_write_u16_le(frame, CCTRL_BUS_SOF);
  frame[2] = CCTRL_BUS_VERSION;
  frame[3] = message_type;
  frame[4] = sequence;
}

size_t cctrl_bus_size(const uint8_t *frame) {
  return (size_t)CCTRL_BUS_HEADER_SIZE + frame[6] + CCTRL_BUS_CRC_SIZE;
}

int cctrl_bus_finalize(uint8_t *frame) {
  const size_t size = cctrl_bus_size(frame);
  if (size > CCTRL_BUS_MAX_FRAME) return 0;
  cctrl_write_u16_le(frame + size - 2, rm_crc16(frame, size - 2));
  return 1;
}

int cctrl_bus_validate_format(const uint8_t *frame, size_t length) {
  if (length < CCTRL_BUS_HEADER_SIZE + CCTRL_BUS_CRC_SIZE ||
      length > CCTRL_BUS_MAX_FRAME) return 0;
  if (cctrl_read_u16_le(frame) != CCTRL_BUS_SOF ||
      frame[2] != CCTRL_BUS_VERSION || cctrl_bus_size(frame) != length) return 0;
  return 1;
}

int cctrl_bus_validate_crc(const uint8_t *frame, size_t length) {
  if (length < CCTRL_BUS_HEADER_SIZE + CCTRL_BUS_CRC_SIZE ||
      length > CCTRL_BUS_MAX_FRAME) return 0;
  return rm_crc16(frame, length - 2) == cctrl_read_u16_le(frame + length - 2);
}

int cctrl_bus_validate(const uint8_t *frame, size_t length) {
  return cctrl_bus_validate_format(frame, length) &&
         cctrl_bus_validate_crc(frame, length);
}

int cctrl_bus_append_record(uint8_t *frame, uint8_t node_id,
                            uint8_t node_type, uint8_t platform,
                            const void *data, uint8_t data_len) {
  const uint8_t record_len = (uint8_t)(sizeof(CctrlNodeRecordHeader) + data_len);
  const uint16_t payload_len = (uint16_t)frame[6] + record_len;
  if (payload_len > CCTRL_BUS_MAX_PAYLOAD) return 0;
  uint8_t *out = frame + CCTRL_BUS_HEADER_SIZE + frame[6];
  out[0] = node_id;
  out[1] = node_type;
  out[2] = platform;
  out[3] = data_len;
  if (data_len != 0 && data != NULL) memcpy(out + 4, data, data_len);
  frame[5]++;
  frame[6] = (uint8_t)payload_len;
  return cctrl_bus_finalize(frame);
}

void cctrl_bus_parser_reset(CctrlBusParser *parser) {
  parser->length = 0;
  parser->expected = 0;
}

int cctrl_bus_parser_push(CctrlBusParser *parser, uint8_t byte) {
  if (parser->length == 0) {
    if (byte != (uint8_t)CCTRL_BUS_SOF) return 0;
    parser->bytes[0] = byte;
    parser->length = 1;
    return 0;
  }
  if (parser->length == 1 && byte != (uint8_t)(CCTRL_BUS_SOF >> 8)) {
    parser->length = byte == (uint8_t)CCTRL_BUS_SOF ? 1 : 0;
    return 0;
  }
  if (parser->length >= CCTRL_BUS_MAX_FRAME) {
    cctrl_bus_parser_reset(parser);
    return -1;
  }
  parser->bytes[parser->length++] = byte;
  if (parser->length == CCTRL_BUS_HEADER_SIZE) {
    const uint16_t expected = CCTRL_BUS_HEADER_SIZE + parser->bytes[6] + CCTRL_BUS_CRC_SIZE;
    if (expected > CCTRL_BUS_MAX_FRAME) {
      cctrl_bus_parser_reset(parser);
      return -2;
    }
    parser->expected = (uint8_t)expected;
  }
  if (parser->expected != 0 && parser->length == parser->expected) {
    return cctrl_bus_validate_crc(parser->bytes, parser->length) ? 1 : -1;
  }
  return 0;
}
