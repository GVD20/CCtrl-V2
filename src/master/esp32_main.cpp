#include "../shared/crc.h"
#include "../shared/protocol_v2.h"
#include "usb_debug_service.h"
#include "master_business.h"
#include "joystick_model.h"
#include "rest_pose.h"
#include "trigger_model.h"
#include "wrist_orientation.h"

#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

namespace {
constexpr uint8_t PIN_CHAIN_TX = 17;
constexpr uint8_t PIN_CHAIN_RX = 18;
constexpr uint8_t PIN_RS232_TX = 15;
constexpr uint8_t PIN_RS232_RX = 16;
constexpr uint8_t KEY_PINS[4] = {35, 36, 37, 38};
constexpr uint32_t BUS_BAUD = 250000;
constexpr uint32_t RS232_BAUD = 115200;
constexpr uint32_t POLL_US = 22222;
constexpr uint32_t HOST_OUTPUT_US = 33334; // <= RoboMaster 30 Hz limit
constexpr uint32_t RESPONSE_TIMEOUT_US = 18000;
constexpr uint32_t LINK_TIMEOUT_MS = 500;
constexpr uint32_t WARNING_HOLD_MS = 2000;
constexpr uint32_t KEY_DOUBLE_CLICK_MS = 300;
constexpr uint32_t KEY_REFRACTORY_MS = 200;
constexpr uint32_t KEY_DEBOUNCE_MS = 20;
constexpr uint32_t KEY_OUTPUT_PULSE_MS = 160;
constexpr uint32_t KEY_AUDIO_PULSE_MS = 80;
constexpr uint32_t ENUM_PERIOD_MS = 500;
constexpr uint32_t POPUP_MIN_MS = 1000;
constexpr uint32_t CAL_MAGIC = 0x364C4143u; // CAL6
constexpr uint8_t CAL_VERSION = 1;
constexpr uint32_t REST_MAGIC = 0x31545352u; // RST1
constexpr uint8_t REST_VERSION = 1;
constexpr uint16_t TRIGGER_S4_ON = 3276;  // 80% of 4095
constexpr uint16_t TRIGGER_S4_OFF = 2048; // release below 50%

struct __attribute__((packed)) CalibrationBlob {
  uint32_t magic;
  uint8_t version;
  uint8_t reserved;
  uint16_t offset[6];
  int8_t direction[6];
  uint16_t crc;
};
static_assert(sizeof(CalibrationBlob) == 26, "calibration blob size");

struct __attribute__((packed)) RestPoseBlob {
  uint32_t magic;
  uint8_t version;
  uint8_t reserved;
  uint16_t raw[6];
  uint16_t revision;
  uint16_t crc;
};
static_assert(sizeof(RestPoseBlob) == 22, "rest pose blob size");

Preferences prefs;
HardwareSerial chainSerial(1);
HardwareSerial rs232Serial(2);
CctrlBusParser busParser{};

uint8_t busSequence;
uint8_t pollSequence;
uint8_t hostSequence;
bool enumerated;
bool awaitingPoll;
uint8_t expectedSequence;
uint8_t encoderSlotByNode[16];
uint8_t nodeTypeByNode[16];
uint8_t totalNodes;
uint8_t observedEncoders;
uint8_t observedHandles;
uint8_t encoderValidMask;
bool handleValid;
uint8_t errorFlags;
uint8_t warningFlags;
uint32_t warningUntilMs[4];
uint8_t systemStatus = CCTRL_SYSTEM_INIT;
uint8_t encoderStatus[6] = {0};
uint16_t encoderRaw[6] = {0};
int16_t encoderCalibrated[6] = {0};
int16_t encoderOutput[6] = {0};
int16_t encoderXzy[6] = {0};
int16_t encoderZxz[6] = {0};
uint32_t encoderSeenMs[6] = {0};
uint16_t handleX, handleY;
int16_t handleMagX, handleMagY, handleMagZ;
uint16_t handleTrigger;
uint8_t handleStatus;
uint8_t handleButtons;
bool triggerS4Active;
uint8_t mainButtonsRaw;
uint8_t mainButtonsHost;
uint8_t mainButtonsAudio;
uint8_t keyDoubleClickFlags;
struct KeyClickState {
  bool sampledDown;
  bool debouncedDown;
  bool wasDown;
  bool firstReleased;
  uint8_t phase;      // 0=ready, 1=wait second press, 2=refractory
  uint8_t outputKind; // 0=none, 1=single, 2=double
  uint32_t sampledChangedMs;
  uint32_t deadlineMs;
  uint32_t outputUntilMs;
  uint32_t audioUntilMs;
  uint32_t doubleToneUntilMs;
};
KeyClickState keyClickState[4];
bool bootSound = true;
bool wristZxzEnabled;
bool restProtectionEnabled = true;
bool startupRestProtectionActive = true;
bool tileMenuActive;
uint8_t tileCommand;
uint32_t linkInitCount;
uint32_t lastEnumMs;
uint32_t lastBusFrameMs;
uint32_t pollSentUs;
uint32_t nextPollUs;
uint32_t nextOutputUs;
uint32_t totalPolls;
uint32_t lostPolls;
uint32_t crcFailures;
uint32_t formatFailures;
uint32_t nodeCountWarnings;
uint32_t magnetWarnings;
uint8_t lastPollSequence;
bool haveLastPollSequence;
uint32_t pollStartedUs[256] = {0};
uint8_t pollTimestampValid[256] = {0};
uint32_t chainLatencyUs;
uint32_t chainLatencyAvgUs;
uint32_t chainLatencyMaxUs;
CalibrationBlob calibration{};
RestPoseBlob restPose{};
RestPose::State restRegionState{};
uint16_t calibrationRevision;
WristOrientation::State wristOrientationState{};
UiPopupState popup{};
uint32_t popupUntilMs;

constexpr uint16_t degreesToCounts(uint16_t degree);
bool encoderCalibrationHealthy(uint8_t status);

void setPopup(const char *text) {
  popup.active = 1;
  strncpy(popup.text, text, sizeof(popup.text) - 1);
  popup.text[sizeof(popup.text) - 1] = 0;
  popupUntilMs = millis() + POPUP_MIN_MS;
}

void setWarning(uint8_t warning) {
  warningFlags |= warning;
  for (uint8_t i = 0; i < 4; ++i)
    if (warning == (uint8_t)(1u << i)) warningUntilMs[i] = millis() + WARNING_HOLD_MS;
}

void refreshWarnings() {
  for (uint8_t i = 0; i < 4; ++i)
    if ((warningFlags & (1u << i)) &&
        (int32_t)(millis() - warningUntilMs[i]) >= 0)
      warningFlags &= (uint8_t)~(1u << i);
}

uint16_t calibrationCrc(const CalibrationBlob &blob) {
  return rm_crc16(reinterpret_cast<const uint8_t *>(&blob),
                  offsetof(CalibrationBlob, crc));
}

uint16_t restPoseCrc(const RestPoseBlob &blob) {
  return rm_crc16(reinterpret_cast<const uint8_t *>(&blob),
                  offsetof(RestPoseBlob, crc));
}

void defaultsCalibration() {
  memset(&calibration, 0, sizeof(calibration));
  calibration.magic = CAL_MAGIC;
  calibration.version = CAL_VERSION;
  for (uint8_t i = 0; i < 6; ++i) calibration.direction[i] = 1;
  calibration.crc = calibrationCrc(calibration);
}

bool loadCalibration() {
  CalibrationBlob candidate{};
  if (prefs.getBytesLength("enc_cal") != sizeof(candidate) ||
      prefs.getBytes("enc_cal", &candidate, sizeof(candidate)) != sizeof(candidate) ||
      candidate.magic != CAL_MAGIC || candidate.version != CAL_VERSION ||
      candidate.crc != calibrationCrc(candidate)) {
    return false;
  }
  for (uint8_t i = 0; i < 6; ++i)
    if (candidate.direction[i] != 1 && candidate.direction[i] != -1) return false;
  calibration = candidate;
  return true;
}

bool storeCalibration() {
  calibration.crc = calibrationCrc(calibration);
  return prefs.putBytes("enc_cal", &calibration, sizeof(calibration)) ==
         sizeof(calibration);
}

uint16_t rawForTarget(uint16_t offset, int8_t direction,
                      uint16_t targetCounts) {
  return direction > 0 ? (uint16_t)((offset + targetCounts) & 0x0FFFu)
                       : (uint16_t)((offset - targetCounts) & 0x0FFFu);
}

void defaultsRestPose() {
  memset(&restPose, 0, sizeof(restPose));
  restPose.magic = REST_MAGIC;
  restPose.version = REST_VERSION;
  const uint16_t target[6] = {0, degreesToCounts(20), degreesToCounts(60),
                              0, 0, 0};
  for (uint8_t i = 0; i < 6; ++i)
    restPose.raw[i] =
        rawForTarget(calibration.offset[i], calibration.direction[i], target[i]);
  restPose.revision = 1;
  restPose.crc = restPoseCrc(restPose);
}

bool loadRestPose() {
  RestPoseBlob candidate{};
  if (prefs.getBytesLength("rest_pose") != sizeof(candidate) ||
      prefs.getBytes("rest_pose", &candidate, sizeof(candidate)) !=
          sizeof(candidate) ||
      candidate.magic != REST_MAGIC || candidate.version != REST_VERSION ||
      candidate.crc != restPoseCrc(candidate))
    return false;
  restPose = candidate;
  return true;
}

bool storeRestPose(const RestPoseBlob &candidate) {
  return prefs.putBytes("rest_pose", &candidate, sizeof(candidate)) ==
         sizeof(candidate);
}

int16_t calibrateRaw(uint16_t raw, uint16_t offset, int8_t direction) {
  uint16_t delta = (raw - offset) & 0x0FFFu;
  if (direction < 0) delta = (uint16_t)((4096u - delta) & 0x0FFFu);
  return delta >= 2048u ? (int16_t)((int32_t)delta - 4096) : (int16_t)delta;
}

void updateCalibrated() {
  for (uint8_t i = 0; i < 6; ++i) {
    encoderCalibrated[i] = calibrateRaw(
        encoderRaw[i], calibration.offset[i], calibration.direction[i]);
  }
  if (!wristZxzEnabled)
    memcpy(encoderOutput, encoderCalibrated, sizeof(encoderOutput));
}

void updateEncoderOutputForHost() {
  int16_t source[6];
  if (startupRestProtectionActive) {
    for (uint8_t i = 0; i < 6; ++i)
      source[i] = calibrateRaw(restPose.raw[i], calibration.offset[i],
                               calibration.direction[i]);
  } else {
    memcpy(source, encoderCalibrated, sizeof(source));
  }
  memcpy(encoderXzy, source, sizeof(encoderXzy));
  encoderXzy[3] = (int16_t)-source[3];
  encoderXzy[5] = (int16_t)-source[5];
  memcpy(encoderZxz, source, sizeof(encoderZxz));
  if (encoderSeenMs[3] != 0 && encoderSeenMs[4] != 0 &&
      encoderSeenMs[5] != 0)
    WristOrientation::xzyToZxz(source + 3, encoderZxz + 3,
                               wristOrientationState);
  memcpy(encoderOutput, wristZxzEnabled ? encoderZxz : source,
         sizeof(encoderOutput));
}

bool restSensorsHealthy() {
  if ((encoderValidMask & 0x1Fu) != 0x1Fu) return false;
  for (uint8_t i = 0; i < 5; ++i)
    if (!encoderCalibrationHealthy(encoderStatus[i])) return false;
  return true;
}

void updateRestProtection(uint32_t nowMs) {
  RestPose::update(restRegionState, encoderRaw, restPose.raw,
                   restSensorsHealthy(), nowMs);
  if (startupRestProtectionActive && restRegionState.in_region)
    startupRestProtectionActive = false;
  if (startupRestProtectionActive) {
    errorFlags |= CCTRL_ERROR_REST_REQUIRED;
    if (systemStatus != CCTRL_SYSTEM_DISCONNECTED)
      systemStatus = CCTRL_SYSTEM_DEGRADED;
  }
}

bool encoderCalibrationHealthy(uint8_t status) {
  const uint8_t required = CCTRL_ENC_I2C_OK | CCTRL_ENC_MAGNET_DETECTED;
  return (status & required) == required;
}

constexpr uint16_t degreesToCounts(uint16_t degree) {
  return (uint16_t)((((uint32_t)(degree % 360u) * 4096u) + 180u) / 360u);
}
static_assert(degreesToCounts(20) == 228, "A2 target count");
static_assert(degreesToCounts(60) == 683, "A3 target count");

uint16_t offsetForTarget(uint16_t raw, int8_t direction,
                         uint16_t targetCounts) {
  return direction > 0 ? (uint16_t)((raw - targetCounts) & 0x0FFFu)
                       : (uint16_t)((raw + targetCounts) & 0x0FFFu);
}

void resetTopology() {
  memset(encoderSlotByNode, 0xFF, sizeof(encoderSlotByNode));
  memset(nodeTypeByNode, 0, sizeof(nodeTypeByNode));
  totalNodes = observedEncoders = observedHandles = 0;
  encoderValidMask = 0;
  handleValid = false;
  enumerated = false;
}

void sendBusFrame(uint8_t messageType) {
  uint8_t frame[CCTRL_BUS_MAX_FRAME];
  cctrl_bus_init(frame, messageType,
                 messageType == CCTRL_MSG_POLL ? pollSequence++ : busSequence++);
  cctrl_bus_finalize(frame);
  if (messageType == CCTRL_MSG_POLL) {
    const uint8_t pollSequence = frame[4];
    pollStartedUs[pollSequence] = micros();
    pollTimestampValid[pollSequence] = 1;
    awaitingPoll = true;
    expectedSequence = pollSequence;
    pollSentUs = micros();
    ++totalPolls;
  }
  chainSerial.write(frame, cctrl_bus_size(frame));
}

void updateChainLatency(uint8_t pollSequence) {
  if (!pollTimestampValid[pollSequence]) return;
  pollTimestampValid[pollSequence] = 0;
  chainLatencyUs = micros() - pollStartedUs[pollSequence];
  if (chainLatencyAvgUs == 0)
    chainLatencyAvgUs = chainLatencyUs;
  else
    chainLatencyAvgUs = (chainLatencyAvgUs * 7u + chainLatencyUs) / 8u;
  if (chainLatencyUs > chainLatencyMaxUs) chainLatencyMaxUs = chainLatencyUs;
}

bool visitRecords(const uint8_t *frame, bool enumeration) {
  uint8_t cursor = CCTRL_BUS_HEADER_SIZE;
  const uint8_t end = (uint8_t)(CCTRL_BUS_HEADER_SIZE + frame[6]);
  uint8_t encoderOrdinal = 0;
  uint8_t recordCount = 0;
  uint8_t newValid = 0;
  bool newHandleValid = false;
  uint16_t seenNodeIds = 0;
  uint8_t tempSlots[16];
  uint8_t tempTypes[16];
  uint8_t tempStatus[6];
  uint16_t tempRaw[6];
  uint32_t tempSeenMs[6];
  uint8_t tempObservedHandles = 0;
  uint8_t tempHandleStatus = 0;
  uint16_t tempHandleX = 0;
  uint16_t tempHandleY = 0;
  int16_t tempMagX = 0;
  int16_t tempMagY = 0;
  int16_t tempMagZ = 0;
  uint8_t tempHandleButtons = 0;
  uint8_t magnetIssueCount = 0;

  if (enumeration) {
    memset(tempSlots, 0xFF, sizeof(tempSlots));
    memset(tempTypes, 0, sizeof(tempTypes));
  } else {
    memcpy(tempSlots, encoderSlotByNode, sizeof(tempSlots));
    memcpy(tempTypes, nodeTypeByNode, sizeof(tempTypes));
    memcpy(tempStatus, encoderStatus, sizeof(tempStatus));
    memcpy(tempRaw, encoderRaw, sizeof(tempRaw));
    memcpy(tempSeenMs, encoderSeenMs, sizeof(tempSeenMs));
  }

  while (cursor < end) {
    if ((uint8_t)(end - cursor) < sizeof(CctrlNodeRecordHeader)) return false;
    const uint8_t nodeId = frame[cursor];
    const uint8_t nodeType = frame[cursor + 1];
    const uint8_t platform = frame[cursor + 2];
    const uint8_t dataLen = frame[cursor + 3];
    if (nodeId >= sizeof(nodeTypeByNode) ||
        (platform != CCTRL_PLATFORM_ATMEGA328P &&
         platform != CCTRL_PLATFORM_CH32V006) ||
        (seenNodeIds & (1u << nodeId)))
      return false;
    seenNodeIds |= (uint16_t)(1u << nodeId);
    cursor += sizeof(CctrlNodeRecordHeader);
    if ((uint8_t)(end - cursor) < dataLen) return false;
    const uint8_t *data = frame + cursor;

    if (enumeration) {
      if (dataLen != sizeof(CctrlEnumData) || nodeId != recordCount) return false;
      if (nodeType == CCTRL_NODE_ENCODER) {
        if (encoderOrdinal >= 6) return false;
        tempSlots[nodeId] = encoderOrdinal;
        tempTypes[nodeId] = CCTRL_NODE_ENCODER;
        ++encoderOrdinal;
      } else if (nodeType == CCTRL_NODE_HANDLE) {
        if (tempObservedHandles != 0) return false;
        tempTypes[nodeId] = CCTRL_NODE_HANDLE;
        ++tempObservedHandles;
      } else {
        return false;
      }
    } else if (nodeType == CCTRL_NODE_ENCODER) {
      if (dataLen != sizeof(CctrlEncoderData) ||
          tempTypes[nodeId] != CCTRL_NODE_ENCODER)
        return false;
      const uint8_t slot = tempSlots[nodeId];
      if (slot >= 6) return false;
      tempStatus[slot] = data[0];
      tempRaw[slot] = cctrl_read_u16_le(data + 1) & 0x0FFFu;
      tempSeenMs[slot] = millis();
      newValid |= (uint8_t)(1u << slot);
      if ((tempStatus[slot] & CCTRL_ENC_MAGNET_DETECTED) == 0 ||
          (tempStatus[slot] &
           (CCTRL_ENC_MAGNET_WEAK | CCTRL_ENC_MAGNET_STRONG)) != 0) {
        ++magnetIssueCount;
      }
    } else if (nodeType == CCTRL_NODE_HANDLE) {
      if (dataLen != sizeof(CctrlHandleData) ||
          tempTypes[nodeId] != CCTRL_NODE_HANDLE || newHandleValid)
        return false;
      tempHandleStatus = data[0];
      tempHandleX = cctrl_read_u16_le(data + 1) & 0x0FFFu;
      tempHandleY = cctrl_read_u16_le(data + 3) & 0x0FFFu;
      tempMagX = (int16_t)cctrl_read_u16_le(data + 5);
      tempMagY = (int16_t)cctrl_read_u16_le(data + 7);
      tempMagZ = (int16_t)cctrl_read_u16_le(data + 9);
      tempHandleButtons = data[11] & 0x0Fu;
      newHandleValid = true;
    } else {
      return false;
    }
    cursor = (uint8_t)(cursor + dataLen);
    ++recordCount;
  }
  if (cursor != end || recordCount != frame[5]) return false;
  if (enumeration) {
    memcpy(encoderSlotByNode, tempSlots, sizeof(encoderSlotByNode));
    memcpy(nodeTypeByNode, tempTypes, sizeof(nodeTypeByNode));
    totalNodes = frame[5];
    observedEncoders = encoderOrdinal;
    observedHandles = tempObservedHandles;
    encoderValidMask = 0;
    handleValid = false;
  } else {
    if (frame[5] != totalNodes) {
      setWarning(CCTRL_WARNING_NODE_COUNT);
      ++nodeCountWarnings;
    }
    memcpy(encoderStatus, tempStatus, sizeof(encoderStatus));
    memcpy(encoderRaw, tempRaw, sizeof(encoderRaw));
    memcpy(encoderSeenMs, tempSeenMs, sizeof(encoderSeenMs));
    encoderValidMask = newValid;
    const uint8_t required = CCTRL_HANDLE_I2C_OK | CCTRL_HANDLE_DATA_READY;
    const bool handleSampleValid =
        newHandleValid && (tempHandleStatus & required) == required &&
        (tempHandleStatus & CCTRL_HANDLE_DATA_STALE) == 0 &&
        (tempMagX != 0 || tempMagY != 0 || tempMagZ != 0);
    handleValid = handleSampleValid;
    if (handleSampleValid) {
      handleStatus = tempHandleStatus;
      handleX = JoystickModel::calibrateX(tempHandleX);
      handleY = JoystickModel::calibrateY(tempHandleY);
      handleMagX = tempMagX;
      handleMagY = tempMagY;
      handleMagZ = tempMagZ;
      handleTrigger = TriggerModel::estimate(handleMagX, handleMagY, handleMagZ);
      if (!triggerS4Active && handleTrigger >= TRIGGER_S4_ON)
        triggerS4Active = true;
      else if (triggerS4Active && handleTrigger < TRIGGER_S4_OFF)
        triggerS4Active = false;
      handleButtons = tempHandleButtons & 0x07u;
      if (triggerS4Active) handleButtons |= 0x08u;
    }
    if (magnetIssueCount) {
      setWarning(CCTRL_WARNING_MAGNET);
      magnetWarnings += magnetIssueCount;
    }
    updateCalibrated();
  }
  return true;
}

void updateSystemState() {
  refreshWarnings();
  errorFlags = 0;
  if (millis() - lastBusFrameMs > LINK_TIMEOUT_MS) {
    errorFlags |= CCTRL_ERROR_TIMEOUT;
    encoderValidMask = 0;
    handleValid = false;
    systemStatus = CCTRL_SYSTEM_DISCONNECTED;
    return;
  }
  systemStatus = enumerated ? CCTRL_SYSTEM_ACTIVE : CCTRL_SYSTEM_INIT;
}

void recordPollSequence(uint8_t pollSequence) {
  if (haveLastPollSequence) {
    const uint8_t delta = (uint8_t)(pollSequence - lastPollSequence);
    if (delta == 0u || delta >= 128u) return;
    if (delta > 1u) lostPolls += (uint8_t)(delta - 1u);
  }
  lastPollSequence = pollSequence;
  haveLastPollSequence = true;
}

bool handleBusFrame(const uint8_t *frame) {
  if (!cctrl_bus_validate_format(frame, cctrl_bus_size(frame))) return false;
  const uint8_t type = frame[3];
  if (type == CCTRL_MSG_ENUM_RESET) {
    if (visitRecords(frame, true)) {
      enumerated = true;
      awaitingPoll = false;
      lastBusFrameMs = millis();
      ++linkInitCount;
      return true;
    } else {
      return false;
    }
  } else if (type == CCTRL_MSG_POLL) {
    if (!enumerated || !visitRecords(frame, false)) {
      return false;
    } else {
      updateChainLatency(frame[4]);
      recordPollSequence(frame[4]);
      lastBusFrameMs = millis();
    }
    if (frame[4] == expectedSequence) awaitingPoll = false;
    return true;
  }
  return false;
}

void serviceBusRx() {
  while (chainSerial.available()) {
    const int result = cctrl_bus_parser_push(&busParser, (uint8_t)chainSerial.read());
    if (result == 1) {
      if (!handleBusFrame(busParser.bytes)) {
        setWarning(CCTRL_WARNING_FORMAT);
        ++formatFailures;
      }
      cctrl_bus_parser_reset(&busParser);
    } else if (result == -1) {
      setWarning(CCTRL_WARNING_CRC);
      ++crcFailures;
      cctrl_bus_parser_reset(&busParser);
    } else if (result == -2) {
      setWarning(CCTRL_WARNING_FORMAT);
      ++formatFailures;
      cctrl_bus_parser_reset(&busParser);
    }
  }
}

void readMainButtons() {
  const uint32_t nowMs = millis();
  uint8_t debounced = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    KeyClickState &state = keyClickState[i];
    const bool sampledDown = digitalRead(KEY_PINS[i]) == LOW;
    if (sampledDown != state.sampledDown) {
      state.sampledDown = sampledDown;
      state.sampledChangedMs = nowMs;
    }
    if (state.debouncedDown != state.sampledDown &&
        (uint32_t)(nowMs - state.sampledChangedMs) >= KEY_DEBOUNCE_MS)
      state.debouncedDown = state.sampledDown;
    if (state.debouncedDown) debounced |= (uint8_t)(1u << i);
  }
  mainButtonsRaw = debounced;

  if (tileMenuActive) {
    for (uint8_t i = 0; i < 4; ++i) {
      KeyClickState &state = keyClickState[i];
      state.wasDown = state.debouncedDown;
      state.firstReleased = false;
      state.phase = 0;
      state.outputKind = 0;
    }
    mainButtonsHost = debounced;
    mainButtonsAudio = debounced;
    keyDoubleClickFlags = 0;
    return;
  }

  uint8_t host = 0;
  uint8_t audio = 0;
  uint8_t doubleTone = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    KeyClickState &state = keyClickState[i];
    const bool down = state.debouncedDown;
    const bool pressed = down && !state.wasDown;
    const bool released = !down && state.wasDown;

    if (state.phase == 1 && (int32_t)(nowMs - state.deadlineMs) >= 0) {
      state.phase = 2;
      state.outputKind = 1;
      state.deadlineMs = nowMs + KEY_REFRACTORY_MS;
      state.outputUntilMs = nowMs + KEY_OUTPUT_PULSE_MS;
    } else if (state.phase == 2 &&
               (int32_t)(nowMs - state.deadlineMs) >= 0) {
      state.phase = 0;
      state.outputKind = 0;
    }

    if (state.phase == 0) {
      if (pressed) {
        state.phase = 1;
        state.firstReleased = false;
        state.deadlineMs = nowMs + KEY_DOUBLE_CLICK_MS;
        state.audioUntilMs = nowMs + KEY_AUDIO_PULSE_MS;
      }
    } else if (state.phase == 1) {
      if (released) state.firstReleased = true;
      if (pressed && state.firstReleased) {
        state.phase = 2;
        state.outputKind = 2;
        state.deadlineMs = nowMs + KEY_REFRACTORY_MS;
        state.outputUntilMs = nowMs + KEY_OUTPUT_PULSE_MS;
        state.audioUntilMs = nowMs + KEY_AUDIO_PULSE_MS;
        state.doubleToneUntilMs = nowMs + KEY_AUDIO_PULSE_MS;
      }
    }

    const bool outputActive =
        state.phase == 2 && (int32_t)(nowMs - state.outputUntilMs) < 0;
    if (outputActive && state.outputKind == 1)
      host |= (uint8_t)(1u << i);
    if (outputActive && state.outputKind == 2)
      host |= (uint8_t)(1u << (i + 4u));
    if ((int32_t)(nowMs - state.audioUntilMs) < 0)
      audio |= (uint8_t)(1u << i);
    if ((int32_t)(nowMs - state.doubleToneUntilMs) < 0)
      doubleTone |= (uint8_t)(1u << i);
    state.wasDown = down;
  }
  mainButtonsHost = host;
  mainButtonsAudio = audio;
  keyDoubleClickFlags = doubleTone;
}

void emitHostFrame() {
  // Advance the stateful wrist conversion at the same 30 Hz cadence seen by
  // the web reference, independent of the 45 Hz Daisy-Chain sampling rate.
  updateEncoderOutputForHost();
  CctrlHostPayloadV2 payload{};
  payload.diagnostic_flags = (uint8_t)((systemStatus & 0x03u) |
                                       ((errorFlags & 0x01u) << 2u) |
                                       ((warningFlags & 0x0Fu) << 3u));
  payload.node_valid_flags = encoderValidMask | (handleValid ? 0x40u : 0u);
  payload.main_buttons = mainButtonsHost;
  payload.handle_buttons = handleButtons;
  memcpy(payload.encoder_status, encoderStatus, sizeof(encoderStatus));
  memcpy(payload.encoder_value, encoderOutput, sizeof(encoderOutput));
  payload.joystick_x = handleX;
  payload.joystick_y = handleY;
  payload.trigger = handleTrigger;

  uint8_t frame[CCTRL_HOST_FRAME_SIZE];
  frame[0] = CCTRL_HOST_SOF;
  cctrl_write_u16_le(frame + 1, CCTRL_HOST_PAYLOAD_SIZE);
  frame[3] = hostSequence++;
  frame[4] = rm_crc8(frame, 4);
  cctrl_write_u16_le(frame + 5, CCTRL_HOST_COMMAND_V2);
  memcpy(frame + 7, &payload, sizeof(payload));
  cctrl_write_u16_le(frame + sizeof(frame) - 2,
                     rm_crc16(frame, sizeof(frame) - 2));
  rs232Serial.write(frame, sizeof(frame));
  tileCommand = 0;
}
} // namespace

namespace MasterBusiness {
void setup() {
  chainSerial.begin(BUS_BAUD, SERIAL_8N1, PIN_CHAIN_RX, PIN_CHAIN_TX);
  rs232Serial.begin(RS232_BAUD, SERIAL_8N1, PIN_RS232_RX, PIN_RS232_TX);
  for (uint8_t pin : KEY_PINS) pinMode(pin, INPUT_PULLUP);
  prefs.begin("cctrl6", false);
  bootSound = prefs.getBool("boot_sound", true);
  wristZxzEnabled = prefs.getBool("wrist_zxz", false);
  restProtectionEnabled = prefs.getBool("rest_guard", true);
  defaultsCalibration();
  loadCalibration();
  calibrationRevision = 1;
  defaultsRestPose();
  loadRestPose();
  startupRestProtectionActive = restProtectionEnabled;
  cctrl_bus_parser_reset(&busParser);
  resetTopology();
  lastBusFrameMs = millis();
  lastEnumMs = millis() - ENUM_PERIOD_MS;
  nextPollUs = nextOutputUs = micros();
}

void loop() {
  serviceBusRx();
  readMainButtons();
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  if ((!enumerated || nowMs - lastBusFrameMs > LINK_TIMEOUT_MS) &&
      nowMs - lastEnumMs >= ENUM_PERIOD_MS) {
    lastEnumMs = nowMs;
    resetTopology();
    sendBusFrame(CCTRL_MSG_ENUM_RESET);
  }
  if (enumerated && !awaitingPoll && (int32_t)(nowUs - nextPollUs) >= 0) {
    nextPollUs += POLL_US;
    sendBusFrame(CCTRL_MSG_POLL);
  }
  if (awaitingPoll && nowUs - pollSentUs > RESPONSE_TIMEOUT_US) {
    awaitingPoll = false;
  }
  updateSystemState();
  updateRestProtection(nowMs);
  if ((int32_t)(nowUs - nextOutputUs) >= 0) {
    nextOutputUs += HOST_OUTPUT_US;
    if ((int32_t)(nowUs - nextOutputUs) >= 0)
      nextOutputUs = nowUs + HOST_OUTPUT_US;
    emitHostFrame();
  }
  UsbDebugService::loop(nowMs);
}

bool getMonitorSnapshot(UiMonitorSnapshot &out) {
  out = UiMonitorSnapshot{};
  out.totalNodes = totalNodes;
  out.observedEnc = observedEncoders;
  out.observedHnd = observedHandles;
  out.errFlags = errorFlags;
  out.warningFlags = warningFlags;
  out.controllerStatus = systemStatus;
  out.nodeValidFlags = encoderValidMask | (handleValid ? 0x40 : 0);
  memcpy(out.encStatus, encoderStatus, sizeof(encoderStatus));
  memcpy(out.enc, encoderOutput, sizeof(encoderOutput));
  memcpy(out.encCalibrated, encoderCalibrated, sizeof(encoderCalibrated));
  memcpy(out.encXzy, encoderXzy, sizeof(encoderXzy));
  memcpy(out.encZxz, encoderZxz, sizeof(encoderZxz));
  memcpy(out.encRaw, encoderRaw, sizeof(encoderRaw));
  memcpy(out.encOffset, calibration.offset, sizeof(calibration.offset));
  memcpy(out.encDirection, calibration.direction, sizeof(calibration.direction));
  out.joystickX = handleX;
  out.joystickY = handleY;
  out.trigger = handleTrigger;
  out.handleButtons = handleButtons;
  out.keyFlags = mainButtonsHost;
  out.keyAudioFlags = mainButtonsAudio;
  out.keyDoubleClickFlags = keyDoubleClickFlags;
  out.wristLimitActive =
      wristZxzEnabled && wristOrientationState.soft_limited &&
              !restRegionState.in_region
          ? 1
          : 0;
  out.disconnectMode = systemStatus == CCTRL_SYSTEM_DISCONNECTED;
  out.restInRegion = restRegionState.in_region ? 1 : 0;
  out.restProtectionEnabled = restProtectionEnabled ? 1 : 0;
  out.restProtectionActive = startupRestProtectionActive ? 1 : 0;
  out.usbDebugEnabled = UsbDebugService::enabled() ? 1 : 0;
  out.usbConnected = UsbDebugService::connected() ? 1 : 0;
  out.lossRate10s = totalPolls ? (100.0f * lostPolls / totalPolls) : 0.0f;
  out.chainLatencyUs = chainLatencyUs;
  out.chainLatencyAvgUs = chainLatencyAvgUs;
  out.chainLatencyMaxUs = chainLatencyMaxUs;
  out.lostPolls = lostPolls;
  out.crcFailures = crcFailures;
  out.formatFailures = formatFailures;
  out.nodeCountWarnings = nodeCountWarnings;
  out.magnetWarnings = magnetWarnings;
  out.valid = 1;
  return true;
}

bool getUiPopupState(UiPopupState &out) {
  if (popup.active && (int32_t)(millis() - popupUntilMs) >= 0)
    popup.active = 0;
  out = popup;
  return true;
}
uint8_t getLocalKeyMaskRaw() { return mainButtonsRaw; }
bool setTileMenuActive(bool active) { tileMenuActive = active; return true; }
bool getTileMenuActive() { return tileMenuActive; }
bool triggerTileMenuCommand(uint8_t command) { tileCommand = command; return true; }

bool zeroAllEncoders() {
  if (encoderValidMask != 0x3Fu) {
    setPopup("Need 6 encoders");
    return false;
  }
  for (uint8_t i = 0; i < 6; ++i) {
    if (!encoderCalibrationHealthy(encoderStatus[i])) {
      setPopup("Encoder not ready");
      return false;
    }
  }
  CalibrationBlob candidate = calibration;
  memcpy(candidate.offset, encoderRaw, sizeof(candidate.offset));
  candidate.offset[1] = offsetForTarget(
      encoderRaw[1], candidate.direction[1], degreesToCounts(20));
  candidate.offset[2] = offsetForTarget(
      encoderRaw[2], candidate.direction[2], degreesToCounts(60));
  candidate.crc = calibrationCrc(candidate);
  if (prefs.putBytes("enc_cal", &candidate, sizeof(candidate)) != sizeof(candidate)) {
    setPopup("NVS write failed");
    return false;
  }
  calibration = candidate;
  ++calibrationRevision;
  updateCalibrated();
  setPopup("Calibrated all 6");
  return true;
}

bool getEncoderCalibrationState(EncoderCalibrationState &out) {
  memcpy(out.raw, encoderRaw, sizeof(out.raw));
  memcpy(out.offset, calibration.offset, sizeof(out.offset));
  memcpy(out.dir, calibration.direction, sizeof(out.dir));
  memcpy(out.calibrated, encoderCalibrated, sizeof(out.calibrated));
  out.revision = calibrationRevision;
  return true;
}

bool calibrateEncoderAxisToDegree(uint8_t axis, uint16_t degree) {
  if (axis >= 6 || !(encoderValidMask & (1u << axis)) ||
      !encoderCalibrationHealthy(encoderStatus[axis])) return false;
  const uint16_t target = degreesToCounts(degree);
  calibration.offset[axis] = offsetForTarget(
      encoderRaw[axis], calibration.direction[axis], target);
  ++calibrationRevision;
  updateCalibrated();
  return true;
}

bool toggleEncoderDirection(uint8_t axis) {
  if (axis >= 6) return false;
  calibration.direction[axis] = -calibration.direction[axis];
  ++calibrationRevision;
  updateCalibrated();
  return true;
}
bool applyEncoderCalibration(const uint16_t offset[6], const int8_t dir[6],
                             uint16_t expectedRevision) {
  if (!offset || !dir || expectedRevision != calibrationRevision) return false;
  CalibrationBlob candidate = calibration;
  for (uint8_t i = 0; i < 6; ++i) {
    if (offset[i] > 4095u || (dir[i] != 1 && dir[i] != -1)) return false;
    candidate.offset[i] = offset[i];
    candidate.direction[i] = dir[i];
  }
  candidate.crc = calibrationCrc(candidate);
  if (prefs.putBytes("enc_cal", &candidate, sizeof(candidate)) !=
      sizeof(candidate))
    return false;
  calibration = candidate;
  ++calibrationRevision;
  updateCalibrated();
  return true;
}
bool saveEncoderCalibration() {
  const bool ok = storeCalibration();
  setPopup(ok ? "Calibration saved" : "Save failed");
  return ok;
}
bool reloadEncoderCalibration() {
  const bool ok = loadCalibration();
  if (ok) {
    ++calibrationRevision;
    updateCalibrated();
  }
  setPopup(ok ? "Calibration loaded" : "Invalid calibration");
  return ok;
}
bool setBootSoundEnabled(bool enabled) { bootSound = enabled; prefs.putBool("boot_sound", enabled); return true; }
bool getBootSoundEnabled() { return bootSound; }
bool setWristZxzEnabled(bool enabled) {
  wristZxzEnabled = enabled;
  WristOrientation::reset(wristOrientationState);
  updateCalibrated();
  prefs.putBool("wrist_zxz", enabled);
  setPopup(enabled ? "Wrist output: ZXZ" : "Wrist output: XZY");
  return true;
}
bool getWristZxzEnabled() { return wristZxzEnabled; }
bool getRestPoseState(RestPoseState &out) {
  memcpy(out.raw, restPose.raw, sizeof(out.raw));
  out.revision = restPose.revision;
  out.inRegion = restRegionState.in_region ? 1 : 0;
  out.protectionEnabled = restProtectionEnabled ? 1 : 0;
  out.protectionActive = startupRestProtectionActive ? 1 : 0;
  return true;
}
bool setCurrentPoseAsRest() {
  if (encoderValidMask != 0x3Fu) return false;
  for (uint8_t i = 0; i < 6; ++i)
    if (!encoderCalibrationHealthy(encoderStatus[i])) return false;
  RestPoseBlob candidate = restPose;
  memcpy(candidate.raw, encoderRaw, sizeof(candidate.raw));
  ++candidate.revision;
  candidate.crc = restPoseCrc(candidate);
  if (!storeRestPose(candidate)) return false;
  restPose = candidate;
  restRegionState = RestPose::State{};
  return true;
}
bool setRestProtectionEnabled(bool enabled) {
  restProtectionEnabled = enabled;
  if (!enabled) startupRestProtectionActive = false;
  prefs.putBool("rest_guard", enabled);
  return true;
}
bool getRestProtectionEnabled() { return restProtectionEnabled; }
bool startUsbDebug() { return UsbDebugService::requestStart(); }
bool getUsbDebugEnabled() { return UsbDebugService::enabled(); }
bool getUsbConnected() { return UsbDebugService::connected(); }
uint32_t getLinkInitSuccessCount() { return linkInitCount; }

} // namespace MasterBusiness
