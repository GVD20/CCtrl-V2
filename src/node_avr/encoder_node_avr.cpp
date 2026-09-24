#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "../shared/protocol_v2.h"

namespace {
constexpr uint8_t LED_ERR = 8;
constexpr uint8_t LED_STAT = 13;
constexpr uint32_t SAMPLE_PERIOD_MS = 2;
constexpr uint16_t CAP_AS5600 = 1u << 0;

AS5600 sensor;
CctrlBusParser parser{};
uint8_t nodeId = 0xFF;
bool enumerated = false;
uint8_t encoderStatus = CCTRL_ENC_DATA_STALE;
uint16_t rawAngle = 0;
uint32_t lastSampleMs = 0;
uint32_t lastGoodSampleMs = 0;
uint32_t crcErrorUntilMs = 0;

void sampleSensor() {
  if (!sensor.isConnected()) {
    encoderStatus = CCTRL_ENC_DATA_STALE;
    return;
  }
  const uint8_t asStatus = sensor.readStatus();
  uint8_t flags = CCTRL_ENC_I2C_OK;
  if (asStatus & 0x20u) flags |= CCTRL_ENC_MAGNET_DETECTED;
  if (asStatus & 0x10u) flags |= CCTRL_ENC_MAGNET_WEAK;
  if (asStatus & 0x08u) flags |= CCTRL_ENC_MAGNET_STRONG;
  if (flags & CCTRL_ENC_MAGNET_DETECTED) {
    rawAngle = sensor.readAngle() & 0x0FFFu;
    lastGoodSampleMs = millis();
  }
  if (millis() - lastGoodSampleMs > 50) flags |= CCTRL_ENC_DATA_STALE;
  encoderStatus = flags;
}

bool pulseTrain(uint32_t now, uint8_t count, uint32_t cycleMs) {
  const uint32_t phase = now % cycleMs;
  return phase < (uint32_t)count * 240u && (phase % 240u) < 120u;
}

void updateLeds() {
  const uint32_t now = millis();
  if (now < 600u) {
    const bool on = now < 120u || (now >= 240u && now < 360u);
    digitalWrite(LED_STAT, on ? HIGH : LOW);
    digitalWrite(LED_ERR, on ? HIGH : LOW);
    return;
  }
  uint8_t errCount = 0;
  if ((int32_t)(crcErrorUntilMs - now) > 0)
    errCount = 4;
  else if ((encoderStatus & CCTRL_ENC_I2C_OK) == 0)
    errCount = 5;
  else if ((encoderStatus & CCTRL_ENC_MAGNET_DETECTED) == 0)
    errCount = 1;
  else if (encoderStatus & CCTRL_ENC_MAGNET_WEAK)
    errCount = 2;
  else if (encoderStatus & CCTRL_ENC_MAGNET_STRONG)
    errCount = 3;
  digitalWrite(LED_ERR, errCount && pulseTrain(now, errCount, 2000u) ? HIGH : LOW);
  digitalWrite(LED_STAT,
               errCount == 0 && enumerated &&
                       pulseTrain(now, (uint8_t)(nodeId + 1u), 5000u)
                   ? HIGH
                   : LOW);
}

void processFrame() {
  uint8_t *frame = parser.bytes;
  const uint8_t type = frame[3];
  if (frame[2] != CCTRL_BUS_VERSION) {
    Serial.write(frame, cctrl_bus_size(frame));
    return;
  }
  if (type == CCTRL_MSG_ENUM_RESET) {
    enumerated = false;
    nodeId = frame[5];
    CctrlEnumData data{1, 0, CAP_AS5600};
    if (cctrl_bus_append_record(frame, nodeId, CCTRL_NODE_ENCODER,
                                CCTRL_PLATFORM_ATMEGA328P, &data, sizeof(data))) {
      enumerated = true;
    }
    Serial.write(frame, cctrl_bus_size(frame));
  } else if (type == CCTRL_MSG_POLL && enumerated) {
    CctrlEncoderData data{encoderStatus, rawAngle};
    cctrl_bus_append_record(frame, nodeId, CCTRL_NODE_ENCODER,
                            CCTRL_PLATFORM_ATMEGA328P, &data, sizeof(data));
    Serial.write(frame, cctrl_bus_size(frame));
  } else {
    Serial.write(frame, cctrl_bus_size(frame));
  }
}
} // namespace

void setup() {
  wdt_disable();
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_STAT, OUTPUT);
  digitalWrite(LED_ERR, LOW);
  digitalWrite(LED_STAT, LOW);
  Serial.begin(250000);
  Wire.begin();
  Wire.setClock(400000);
  sensor.begin();
  cctrl_bus_parser_reset(&parser);
  sampleSensor();
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  while (Serial.available()) {
    const int result = cctrl_bus_parser_push(&parser, (uint8_t)Serial.read());
    if (result == 1) {
      processFrame();
      cctrl_bus_parser_reset(&parser);
    } else if (result == -1) {
      crcErrorUntilMs = millis() + 2000;
      cctrl_bus_parser_reset(&parser);
    } else if (result == -2) {
      cctrl_bus_parser_reset(&parser);
    }
  }
  const uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    sampleSensor();
  }
  updateLeds();
}
