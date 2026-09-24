#ifndef CCTRL_PROTOCOL_V2_H
#define CCTRL_PROTOCOL_V2_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CCTRL_BUS_SOF 0xC65Au
#define CCTRL_BUS_VERSION 2u
#define CCTRL_BUS_MAX_FRAME 96u
#define CCTRL_BUS_HEADER_SIZE 8u
#define CCTRL_BUS_CRC_SIZE 2u
#define CCTRL_BUS_MAX_PAYLOAD (CCTRL_BUS_MAX_FRAME - CCTRL_BUS_HEADER_SIZE - CCTRL_BUS_CRC_SIZE)

#define CCTRL_HOST_SOF 0xA5u
#define CCTRL_HOST_COMMAND_V2 0x0302u
#define CCTRL_HOST_PAYLOAD_SIZE 30u
#define CCTRL_HOST_BODY_SIZE (2u + CCTRL_HOST_PAYLOAD_SIZE)
#define CCTRL_HOST_FRAME_SIZE (5u + CCTRL_HOST_BODY_SIZE + 2u)

typedef enum {
  CCTRL_MSG_ENUM_RESET = 0x01,
  CCTRL_MSG_POLL = 0x02,
} CctrlMessageType;

typedef enum {
  CCTRL_NODE_ENCODER = 1,
  CCTRL_NODE_HANDLE = 2,
} CctrlNodeType;

typedef enum {
  CCTRL_PLATFORM_ATMEGA328P = 1,
  CCTRL_PLATFORM_CH32V006 = 2,
} CctrlPlatform;

enum {
  CCTRL_ENC_I2C_OK = 1u << 0,
  CCTRL_ENC_MAGNET_DETECTED = 1u << 1,
  CCTRL_ENC_MAGNET_WEAK = 1u << 2,
  CCTRL_ENC_MAGNET_STRONG = 1u << 3,
  CCTRL_ENC_DATA_STALE = 1u << 4,
};

enum {
  CCTRL_HANDLE_I2C_OK = 1u << 0,
  CCTRL_HANDLE_DATA_READY = 1u << 1,
  CCTRL_HANDLE_DATA_STALE = 1u << 2,
};

enum {
  CCTRL_ERROR_TIMEOUT = 1u << 0,
  CCTRL_ERROR_REST_REQUIRED = 1u << 1,
};

enum {
  CCTRL_WARNING_CRC = 1u << 0,
  CCTRL_WARNING_FORMAT = 1u << 1,
  CCTRL_WARNING_MAGNET = 1u << 2,
  CCTRL_WARNING_NODE_COUNT = 1u << 3,
};

enum {
  CCTRL_SYSTEM_INIT = 0,
  CCTRL_SYSTEM_ACTIVE = 1,
  CCTRL_SYSTEM_DEGRADED = 2,
  CCTRL_SYSTEM_DISCONNECTED = 3,
};

#if defined(__GNUC__)
#define CCTRL_PACKED __attribute__((packed))
#else
#define CCTRL_PACKED
#pragma pack(push, 1)
#endif

typedef struct CCTRL_PACKED {
  uint16_t sof;
  uint8_t version;
  uint8_t message_type;
  uint8_t sequence;
  uint8_t node_count;
  uint8_t payload_len;
  uint8_t flags;
} CctrlBusFrameHeader;

typedef struct CCTRL_PACKED {
  uint8_t node_id;
  uint8_t node_type;
  uint8_t platform;
  uint8_t data_len;
} CctrlNodeRecordHeader;

typedef struct CCTRL_PACKED {
  uint8_t fw_major;
  uint8_t fw_minor;
  uint16_t capabilities;
} CctrlEnumData;

typedef struct CCTRL_PACKED {
  uint8_t status_flags;
  uint16_t raw_angle;
} CctrlEncoderData;

typedef struct CCTRL_PACKED {
  uint8_t status_flags;
  uint16_t joystick_x;
  uint16_t joystick_y;
  int16_t magnetic_x;
  int16_t magnetic_y;
  int16_t magnetic_z;
  uint8_t buttons;
} CctrlHandleData;

typedef struct CCTRL_PACKED {
  uint8_t diagnostic_flags;
  uint8_t node_valid_flags;
  uint8_t main_buttons;
  uint8_t handle_buttons;
  uint8_t encoder_status[6];
  int16_t encoder_value[6];
  uint16_t joystick_x;
  uint16_t joystick_y;
  uint16_t trigger;
  uint16_t reserved;
} CctrlHostPayloadV2;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

typedef struct {
  uint8_t bytes[CCTRL_BUS_MAX_FRAME];
  uint8_t length;
  uint8_t expected;
} CctrlBusParser;

void cctrl_bus_init(uint8_t *frame, uint8_t message_type, uint8_t sequence);
size_t cctrl_bus_size(const uint8_t *frame);
int cctrl_bus_finalize(uint8_t *frame);
int cctrl_bus_validate(const uint8_t *frame, size_t length);
int cctrl_bus_validate_format(const uint8_t *frame, size_t length);
int cctrl_bus_validate_crc(const uint8_t *frame, size_t length);
int cctrl_bus_append_record(uint8_t *frame, uint8_t node_id,
                            uint8_t node_type, uint8_t platform,
                            const void *data, uint8_t data_len);
void cctrl_bus_parser_reset(CctrlBusParser *parser);
int cctrl_bus_parser_push(CctrlBusParser *parser, uint8_t byte);
uint16_t cctrl_read_u16_le(const uint8_t *p);
void cctrl_write_u16_le(uint8_t *p, uint16_t value);

#if defined(__cplusplus)
static_assert(sizeof(CctrlBusFrameHeader) == 8, "bus header size");
static_assert(sizeof(CctrlNodeRecordHeader) == 4, "node record header size");
static_assert(sizeof(CctrlEncoderData) == 3, "encoder data size");
static_assert(sizeof(CctrlHandleData) == 12, "handle data size");
static_assert(sizeof(CctrlHostPayloadV2) == CCTRL_HOST_PAYLOAD_SIZE,
              "RM 0x0302 data must be 30 bytes");
static_assert(CCTRL_HOST_BODY_SIZE == 32, "cmd_id plus data size");
static_assert(CCTRL_HOST_FRAME_SIZE == 39, "complete RM frame size");
#else
_Static_assert(sizeof(CctrlBusFrameHeader) == 8, "bus header size");
_Static_assert(sizeof(CctrlNodeRecordHeader) == 4, "node record header size");
_Static_assert(sizeof(CctrlEncoderData) == 3, "encoder data size");
_Static_assert(sizeof(CctrlHandleData) == 12, "handle data size");
_Static_assert(sizeof(CctrlHostPayloadV2) == CCTRL_HOST_PAYLOAD_SIZE,
               "RM 0x0302 data must be 30 bytes");
_Static_assert(CCTRL_HOST_BODY_SIZE == 32, "cmd_id plus data size");
_Static_assert(CCTRL_HOST_FRAME_SIZE == 39, "complete RM frame size");
#endif

#ifdef __cplusplus
}
#endif
#endif
