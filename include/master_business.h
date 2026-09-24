#pragma once

#include <stddef.h>
#include <stdint.h>

enum : uint8_t {
  MB_CTRL_STATUS_IDLE = 0,
  MB_CTRL_STATUS_ACTIVE = 1,
  MB_CTRL_STATUS_DEGRADED = 2,
  MB_CTRL_STATUS_NODE_DISCONNECT = 3,
};

struct UiMonitorSnapshot {
  uint8_t totalNodes = 0;
  uint8_t expectedEnc = 6;
  uint8_t expectedHnd = 0;
  uint8_t observedEnc = 0;
  uint8_t observedHnd = 0;
  uint8_t errFlags = 0;
  uint8_t warningFlags = 0;
  uint8_t controllerStatus = MB_CTRL_STATUS_IDLE;
  uint8_t nodeValidFlags = 0;
  uint8_t encStatus[6] = {0};
  int16_t enc[6] = {0};
  int16_t encCalibrated[6] = {0};
  int16_t encXzy[6] = {0};
  int16_t encZxz[6] = {0};
  uint16_t encRaw[6] = {0};
  uint16_t encOffset[6] = {0};
  int8_t encDirection[6] = {1, 1, 1, 1, 1, 1};
  uint16_t joystickX = 0;
  uint16_t joystickY = 0;
  uint16_t trigger = 0;
  uint8_t handleButtons = 0;
  uint8_t keyFlags = 0;
  uint8_t keyAudioFlags = 0;
  uint8_t keyDoubleClickFlags = 0;
  uint8_t wristLimitActive = 0;
  uint8_t disconnectMode = 0;
  uint8_t disconnectNodeKnown = 0;
  uint8_t disconnectNodeId = 0xFF;
  uint8_t restInRegion = 0;
  uint8_t restProtectionEnabled = 1;
  uint8_t restProtectionActive = 0;
  uint8_t usbDebugEnabled = 0;
  uint8_t usbConnected = 0;
  float lossRate10s = 0.0f;
  uint32_t chainLatencyUs = 0;
  uint32_t chainLatencyAvgUs = 0;
  uint32_t chainLatencyMaxUs = 0;
  uint32_t lostPolls = 0;
  uint32_t crcFailures = 0;
  uint32_t formatFailures = 0;
  uint32_t nodeCountWarnings = 0;
  uint32_t magnetWarnings = 0;
  uint8_t valid = 0;
};

struct EncoderCalibrationState {
  uint16_t raw[6] = {0};
  uint16_t offset[6] = {0};
  int8_t dir[6] = {1, 1, 1, 1, 1, 1};
  int16_t calibrated[6] = {0};
  uint16_t revision = 0;
};

struct RestPoseState {
  uint16_t raw[6] = {0};
  uint16_t revision = 0;
  uint8_t inRegion = 0;
  uint8_t protectionEnabled = 1;
  uint8_t protectionActive = 0;
};

struct UiPopupState { uint8_t active = 0; char text[32] = {0}; };

namespace MasterBusiness {
void setup();
void loop();
bool getMonitorSnapshot(UiMonitorSnapshot &out);
bool getUiPopupState(UiPopupState &out);
uint8_t getLocalKeyMaskRaw();
bool setTileMenuActive(bool active);
bool getTileMenuActive();
bool triggerTileMenuCommand(uint8_t command);
bool zeroAllEncoders();
bool getEncoderCalibrationState(EncoderCalibrationState &out);
bool calibrateEncoderAxisToDegree(uint8_t axis, uint16_t degree);
bool toggleEncoderDirection(uint8_t axis);
bool applyEncoderCalibration(const uint16_t offset[6], const int8_t dir[6],
                             uint16_t expectedRevision);
bool saveEncoderCalibration();
bool reloadEncoderCalibration();
bool setBootSoundEnabled(bool enabled);
bool getBootSoundEnabled();
bool setWristZxzEnabled(bool enabled);
bool getWristZxzEnabled();
bool getRestPoseState(RestPoseState &out);
bool setCurrentPoseAsRest();
bool setRestProtectionEnabled(bool enabled);
bool getRestProtectionEnabled();
bool startUsbDebug();
bool getUsbDebugEnabled();
bool getUsbConnected();
uint32_t getLinkInitSuccessCount();

} // namespace MasterBusiness
