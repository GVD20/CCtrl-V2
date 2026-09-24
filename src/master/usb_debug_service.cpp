#include "usb_debug_service.h"

#include "../shared/crc.h"
#include "master_business.h"

#include <Arduino.h>
#include <USB.h>
#include <USBCDC.h>
#include <string.h>

namespace UsbDebugService {
namespace {

constexpr uint8_t SOF0 = 0x43;
constexpr uint8_t SOF1 = 0x44;
constexpr uint8_t VERSION = 1;
constexpr uint8_t TYPE_RAW = 1;
constexpr uint8_t TYPE_XZY = 2;
constexpr uint8_t TYPE_ZXZ = 3;
constexpr uint8_t TYPE_DIAG = 4;
constexpr uint8_t TYPE_CONFIG = 5;
constexpr uint8_t TYPE_REST = 6;
constexpr uint8_t TYPE_RESULT = 7;
constexpr uint8_t TYPE_COMMAND = 0x80;
constexpr uint8_t OP_GET_STATE = 0x01;
constexpr uint8_t OP_APPLY_CONFIG = 0x10;
constexpr uint8_t OP_SET_REST_CURRENT = 0x20;
constexpr uint8_t OP_SET_REST_PROTECTION = 0x21;
constexpr uint32_t TELEMETRY_PERIOD_MS = 67;
constexpr size_t MAX_PAYLOAD = 32;
constexpr size_t HEADER_SIZE = 6;

USBCDC debugSerial;
volatile bool startRequested;
bool actualEnabled;
uint8_t txSequence;
uint32_t lastTelemetryMs;
uint8_t rxFrame[HEADER_SIZE + MAX_PAYLOAD + 2];
size_t rxLength;

void writeU16(uint8_t *out, uint16_t value) {
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8);
}

uint16_t readU16(const uint8_t *in) {
  return (uint16_t)(in[0] | ((uint16_t)in[1] << 8));
}

void sendFrame(uint8_t type, const uint8_t *payload, uint8_t length) {
  if (!actualEnabled || length > MAX_PAYLOAD) return;
  uint8_t frame[HEADER_SIZE + MAX_PAYLOAD + 2];
  frame[0] = SOF0;
  frame[1] = SOF1;
  frame[2] = VERSION;
  frame[3] = type;
  frame[4] = txSequence++;
  frame[5] = length;
  if (length) memcpy(frame + HEADER_SIZE, payload, length);
  const size_t crcOffset = HEADER_SIZE + length;
  writeU16(frame + crcOffset, rm_crc16(frame, crcOffset));
  debugSerial.write(frame, crcOffset + 2);
}

void sendConfig() {
  EncoderCalibrationState state{};
  MasterBusiness::getEncoderCalibrationState(state);
  uint8_t payload[15]{};
  writeU16(payload, state.revision);
  for (uint8_t i = 0; i < 6; ++i) writeU16(payload + 2 + i * 2, state.offset[i]);
  for (uint8_t i = 0; i < 6; ++i)
    if (state.dir[i] < 0) payload[14] |= (uint8_t)(1u << i);
  sendFrame(TYPE_CONFIG, payload, sizeof(payload));
}

void sendRest() {
  RestPoseState state{};
  MasterBusiness::getRestPoseState(state);
  uint8_t payload[15]{};
  writeU16(payload, state.revision);
  for (uint8_t i = 0; i < 6; ++i) writeU16(payload + 2 + i * 2, state.raw[i]);
  payload[14] = (uint8_t)((state.inRegion ? 1u : 0u) |
                          (state.protectionEnabled ? 2u : 0u) |
                          (state.protectionActive ? 4u : 0u));
  sendFrame(TYPE_REST, payload, sizeof(payload));
}

void sendResult(uint8_t operation, bool ok) {
  EncoderCalibrationState calibration{};
  RestPoseState rest{};
  MasterBusiness::getEncoderCalibrationState(calibration);
  MasterBusiness::getRestPoseState(rest);
  uint8_t payload[7]{};
  payload[0] = operation;
  payload[1] = ok ? 0 : 1;
  writeU16(payload + 2, calibration.revision);
  writeU16(payload + 4, rest.revision);
  payload[6] = (uint8_t)((rest.inRegion ? 1u : 0u) |
                         (rest.protectionEnabled ? 2u : 0u) |
                         (rest.protectionActive ? 4u : 0u));
  sendFrame(TYPE_RESULT, payload, sizeof(payload));
}

void handleCommand(const uint8_t *payload, uint8_t length) {
  if (!length) return;
  const uint8_t operation = payload[0];
  bool ok = false;
  if (operation == OP_GET_STATE && length == 1) {
    ok = true;
  } else if (operation == OP_APPLY_CONFIG && length == 16) {
    const uint16_t expectedRevision = readU16(payload + 1);
    uint16_t offset[6];
    int8_t direction[6];
    for (uint8_t i = 0; i < 6; ++i) {
      offset[i] = readU16(payload + 3 + i * 2);
      direction[i] = (payload[15] & (1u << i)) ? -1 : 1;
    }
    ok = MasterBusiness::applyEncoderCalibration(offset, direction,
                                                  expectedRevision);
  } else if (operation == OP_SET_REST_CURRENT && length == 1) {
    ok = MasterBusiness::setCurrentPoseAsRest();
  } else if (operation == OP_SET_REST_PROTECTION && length == 2) {
    ok = MasterBusiness::setRestProtectionEnabled(payload[1] != 0);
  }
  sendConfig();
  sendRest();
  sendResult(operation, ok);
}

void consumeByte(uint8_t value) {
  if (rxLength == 0 && value != SOF0) return;
  if (rxLength == 1 && value != SOF1) {
    rxLength = value == SOF0 ? 1 : 0;
    return;
  }
  rxFrame[rxLength++] = value;
  if (rxLength < HEADER_SIZE) return;
  const uint8_t payloadLength = rxFrame[5];
  if (payloadLength > MAX_PAYLOAD) {
    rxLength = 0;
    return;
  }
  const size_t total = HEADER_SIZE + payloadLength + 2;
  if (rxLength < total) return;
  if (rxFrame[2] == VERSION && rxFrame[3] == TYPE_COMMAND &&
      readU16(rxFrame + total - 2) == rm_crc16(rxFrame, total - 2))
    handleCommand(rxFrame + HEADER_SIZE, payloadLength);
  rxLength = 0;
}

void serviceRx() {
  while (debugSerial.available()) consumeByte((uint8_t)debugSerial.read());
}

void sendTelemetry() {
  UiMonitorSnapshot snapshot{};
  if (!MasterBusiness::getMonitorSnapshot(snapshot)) return;
  uint8_t pose[13];
  pose[0] = snapshot.nodeValidFlags;
  for (uint8_t i = 0; i < 6; ++i) writeU16(pose + 1 + i * 2, snapshot.encRaw[i]);
  sendFrame(TYPE_RAW, pose, sizeof(pose));
  for (uint8_t i = 0; i < 6; ++i)
    writeU16(pose + 1 + i * 2, (uint16_t)snapshot.encXzy[i]);
  sendFrame(TYPE_XZY, pose, sizeof(pose));
  for (uint8_t i = 0; i < 6; ++i)
    writeU16(pose + 1 + i * 2, (uint16_t)snapshot.encZxz[i]);
  sendFrame(TYPE_ZXZ, pose, sizeof(pose));

  uint8_t diag[17]{};
  diag[0] = snapshot.controllerStatus;
  diag[1] = snapshot.errFlags;
  diag[2] = snapshot.warningFlags;
  diag[3] = snapshot.nodeValidFlags;
  diag[4] = snapshot.keyFlags;
  diag[5] = snapshot.handleButtons;
  diag[6] = (uint8_t)((snapshot.restInRegion ? 1u : 0u) |
                      (snapshot.restProtectionEnabled ? 2u : 0u) |
                      (snapshot.restProtectionActive ? 4u : 0u) |
                      (snapshot.wristLimitActive ? 8u : 0u));
  writeU16(diag + 7, snapshot.joystickX);
  writeU16(diag + 9, snapshot.joystickY);
  writeU16(diag + 11, snapshot.trigger);
  writeU16(diag + 13, snapshot.chainLatencyUs > 65535u
                                ? 65535u
                                : (uint16_t)snapshot.chainLatencyUs);
  writeU16(diag + 15, snapshot.lostPolls > 65535u
                                ? 65535u
                                : (uint16_t)snapshot.lostPolls);
  sendFrame(TYPE_DIAG, diag, sizeof(diag));
}

void start() {
  debugSerial.setTxTimeoutMs(0);
  debugSerial.begin(2000000);
  USB.productName("CCtrl USB Debug");
  USB.manufacturerName("CCtrl");
  actualEnabled = USB.begin();
}

} // namespace

bool requestStart() {
  startRequested = true;
  return true;
}

bool enabled() { return actualEnabled; }
bool connected() { return actualEnabled && (bool)debugSerial; }

void loop(uint32_t nowMs) {
  if (startRequested && !actualEnabled) {
    startRequested = false;
    start();
  }
  if (!actualEnabled) return;
  serviceRx();
  if ((uint32_t)(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    sendTelemetry();
  }
}

} // namespace UsbDebugService
