#include <Arduino.h>

#include "ice_melody_data.h"
#include <WouoUiLiteGeneralBridge.h>
#include <master_business.h>
#include "../shared/protocol_v2.h"
#include <tile_menu_audio.h>

namespace {
constexpr uint8_t PIN_LED_ERR = 42;
constexpr uint8_t PIN_LED_STAT = 41;
constexpr uint8_t PIN_BUZZER = 2;
constexpr uint8_t BUZZER_PWM_RES_BITS = 8;
constexpr uint8_t BUZZER_PWM_DUTY_BOOT = 51;  // about 30% on 8-bit PWM
constexpr uint8_t BUZZER_PWM_DUTY_ERROR = 51; // about 20% on 8-bit PWM
constexpr uint8_t BUZZER_PWM_DUTY_LINK_OK = 64;
constexpr uint8_t BUZZER_PWM_DUTY_KEY_PRESS = 64;
constexpr uint8_t BUZZER_PWM_DUTY_WRIST_LIMIT = 38;
constexpr uint16_t BOOT_MELODY_BASE_BPM = 120;
constexpr uint16_t BOOT_MELODY_TARGET_BPM = 195;
constexpr uint32_t BOOT_MELODY_STOP_GUARD_MS = 1500;

constexpr uint8_t BUZZER_PWM_CHANNEL = 0;

enum BuzzerSoundKind : uint8_t {
  BUZZER_SOUND_NONE = 0,
  BUZZER_SOUND_BOOT,
  BUZZER_SOUND_ERROR,
  BUZZER_SOUND_LINK_OK,
  BUZZER_SOUND_KEY_PRESS,
  BUZZER_SOUND_KEY_RELEASE,
  BUZZER_SOUND_TILE_NAV_BACKWARD,
  BUZZER_SOUND_TILE_NAV_FORWARD,
  BUZZER_SOUND_TILE_CONFIRM,
  BUZZER_SOUND_WRIST_LIMIT,
};

struct BuzzerPlaybackState {
  const BuzzerPwmStep *steps = nullptr;
  size_t count = 0;
  size_t index = 0;
  uint8_t duty = 0;
  BuzzerSoundKind soundKind = BUZZER_SOUND_NONE;
  bool active = false;
  bool ownsPwm = false;
  bool repeat = false;
  bool tempoScale = false;
  uint32_t deadlineMs = 0;
  uint32_t startMs = 0;
  uint32_t guardMs = 0;
};

static constexpr BuzzerPwmStep kErrorBeepPattern[] = {
    {1760, 70},
    {0, 50},
    {1760, 90},
    {0, 1000},
};
static constexpr size_t kErrorBeepPatternCount =
    sizeof(kErrorBeepPattern) / sizeof(kErrorBeepPattern[0]);
static constexpr uint32_t kErrorBeepGuardMs = 1500;
static constexpr BuzzerPwmStep kLinkOkPattern[] = {
    {523, 70}, {0, 25}, {659, 70}, {0, 25}, {784, 95},
};
static constexpr size_t kLinkOkPatternCount =
    sizeof(kLinkOkPattern) / sizeof(kLinkOkPattern[0]);
static constexpr uint32_t kLinkOkGuardMs = 450;
static constexpr BuzzerPwmStep kKeyPressPattern[] = {
    {988, 35},
};
static constexpr size_t kKeyPressPatternCount =
    sizeof(kKeyPressPattern) / sizeof(kKeyPressPattern[0]);
static constexpr uint32_t kKeyPressGuardMs = 80;
static constexpr BuzzerPwmStep kKeyDoubleClickPattern[] = {
    {1319, 35},
};
static constexpr size_t kKeyDoubleClickPatternCount =
    sizeof(kKeyDoubleClickPattern) / sizeof(kKeyDoubleClickPattern[0]);
static constexpr BuzzerPwmStep kKeyReleasePattern[] = {
    {494, 45},
};
static constexpr size_t kKeyReleasePatternCount =
    sizeof(kKeyReleasePattern) / sizeof(kKeyReleasePattern[0]);
static constexpr uint32_t kKeyReleaseGuardMs = 90;
static constexpr BuzzerPwmStep kTileNavBackwardPattern[] = {
    {523, 26},
};
static constexpr size_t kTileNavBackwardPatternCount =
    sizeof(kTileNavBackwardPattern) / sizeof(kTileNavBackwardPattern[0]);
static constexpr uint32_t kTileNavBackwardGuardMs = 80;
static constexpr BuzzerPwmStep kTileNavForwardPattern[] = {
    {1047, 26},
};
static constexpr size_t kTileNavForwardPatternCount =
    sizeof(kTileNavForwardPattern) / sizeof(kTileNavForwardPattern[0]);
static constexpr uint32_t kTileNavForwardGuardMs = 80;
static constexpr BuzzerPwmStep kTileConfirmPattern[] = {
    {523, 35},
    {0, 12},
    {659, 35},
    {0, 12},
    {784, 45},
};
static constexpr size_t kTileConfirmPatternCount =
    sizeof(kTileConfirmPattern) / sizeof(kTileConfirmPattern[0]);
static constexpr uint32_t kTileConfirmGuardMs = 180;
static constexpr BuzzerPwmStep kWristLimitPattern[] = {
    {1047, 100},
    {784, 150},
    {0, 280},
    {1047, 100},
    {784, 180},
    {0, 3000},
};
static constexpr size_t kWristLimitPatternCount =
    sizeof(kWristLimitPattern) / sizeof(kWristLimitPattern[0]);
static constexpr uint32_t kWristLimitGuardMs = 1400;

static uint32_t scaleDurationForTempo(uint32_t durationMs) {
  if (durationMs == 0U || BOOT_MELODY_BASE_BPM == 0U ||
      BOOT_MELODY_TARGET_BPM == 0U) {
    return durationMs;
  }

  uint64_t scaled = (uint64_t)durationMs * (uint64_t)BOOT_MELODY_BASE_BPM +
                    (uint64_t)(BOOT_MELODY_TARGET_BPM / 2U);
  scaled /= (uint64_t)BOOT_MELODY_TARGET_BPM;
  if (scaled == 0U) {
    scaled = 1U;
  }
  return (uint32_t)scaled;
}

bool gBuzzerPwmReady = false;
uint32_t gLastLinkInitSuccessCount = 0;
bool gLastErrorState = false;
uint8_t gLastKeyFlags = 0;
uint8_t gLastDoubleClickFlags = 0;
uint8_t gLastHandleButtons = 0;
volatile bool gBootBuzzerStartRequested = false;
BuzzerPlaybackState gBuzzerPlayback;

static void startBootBuzzer();
static void startErrorBuzzer();
static void startLinkOkBuzzer();
static void startKeyPressBuzzer();
static void startKeyReleaseBuzzer();
static void startWristLimitBuzzer();
static void stopBuzzerPlayback();

static void buzzerMute() {
#if defined(ARDUINO_ARCH_ESP32)
  if (gBuzzerPwmReady) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
#else
    ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
#endif
  }
#endif
  digitalWrite(PIN_BUZZER, LOW);
}

static void buzzerShutdown() {
#if defined(ARDUINO_ARCH_ESP32)
  if (gBuzzerPwmReady) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
    ledcDetach(PIN_BUZZER);
#else
    ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
    ledcDetachPin(PIN_BUZZER);
#endif
  }
#endif
  gBuzzerPwmReady = false;
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
}

static bool buzzerInitPwm() {
  if (gBuzzerPwmReady) {
    return true;
  }

#if defined(ARDUINO_ARCH_ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  if (ledcAttach(PIN_BUZZER, 1000, BUZZER_PWM_RES_BITS)) {
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
    return true;
  }
#else
  ledcSetup(BUZZER_PWM_CHANNEL, 1000, BUZZER_PWM_RES_BITS);
  ledcAttachPin(PIN_BUZZER, BUZZER_PWM_CHANNEL);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
  return true;
#endif
#endif
  return false;
}

static void buzzerSetTone(uint16_t freqHz, uint8_t duty) {
  if (!gBuzzerPwmReady || freqHz == 0U) {
    buzzerMute();
    return;
  }

#if defined(ARDUINO_ARCH_ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(PIN_BUZZER, freqHz);
  ledcWrite(PIN_BUZZER, duty);
#else
  ledcWriteTone(BUZZER_PWM_CHANNEL, freqHz);
  ledcWrite(BUZZER_PWM_CHANNEL, duty);
#endif
#endif
}

static void stopBuzzerPlayback() {
  gBuzzerPlayback.active = false;
  gBuzzerPlayback.steps = nullptr;
  gBuzzerPlayback.count = 0;
  gBuzzerPlayback.index = 0;
  gBuzzerPlayback.duty = 0;
  gBuzzerPlayback.soundKind = BUZZER_SOUND_NONE;
  gBuzzerPlayback.deadlineMs = 0;
  gBuzzerPlayback.startMs = 0;
  gBuzzerPlayback.guardMs = 0;
  gBuzzerPlayback.repeat = false;
  gBuzzerPlayback.tempoScale = false;
  if (gBuzzerPlayback.ownsPwm || gBuzzerPwmReady) {
    buzzerShutdown();
  }
  gBuzzerPlayback.ownsPwm = false;
}

static void startBuzzerPlayback(const BuzzerPwmStep *steps, size_t count,
                                uint8_t duty, uint32_t guardMs, bool tempoScale,
                                bool repeat, BuzzerSoundKind soundKind) {
  stopBuzzerPlayback();
  if (!steps || count == 0U) {
    return;
  }

  gBuzzerPwmReady = buzzerInitPwm();
  if (!gBuzzerPwmReady) {
    return;
  }

  gBuzzerPlayback.steps = steps;
  gBuzzerPlayback.count = count;
  gBuzzerPlayback.index = 0;
  gBuzzerPlayback.duty = duty;
  gBuzzerPlayback.soundKind = soundKind;
  gBuzzerPlayback.active = true;
  gBuzzerPlayback.ownsPwm = true;
  gBuzzerPlayback.startMs = millis();
  gBuzzerPlayback.guardMs = guardMs;
  gBuzzerPlayback.repeat = repeat;
  gBuzzerPlayback.tempoScale = tempoScale;
}

static bool isBootBuzzerActive() {
  return gBuzzerPlayback.active &&
         gBuzzerPlayback.soundKind == BUZZER_SOUND_BOOT;
}

static void requestPromptPlayback(const BuzzerPwmStep *steps, size_t count,
                                  uint8_t duty, uint32_t guardMs, bool repeat,
                                  BuzzerSoundKind soundKind) {
  // Boot melody owns the buzzer until it finishes so power-on audio cannot be
  // cut off by transient prompt events during startup.
  if (isBootBuzzerActive()) {
    return;
  }
  startBuzzerPlayback(steps, count, duty, guardMs, false, repeat, soundKind);
  if (gBuzzerPlayback.active) {
    buzzerSetTone(0, 0);
    gBuzzerPlayback.deadlineMs = gBuzzerPlayback.startMs;
  }
}

static void advanceBuzzerPlayback(uint32_t nowMs, bool errorState) {
  while (gBuzzerPlayback.active) {
    if (gBuzzerPlayback.index >= gBuzzerPlayback.count) {
      if (gBuzzerPlayback.repeat &&
          gBuzzerPlayback.soundKind == BUZZER_SOUND_ERROR && errorState) {
        gBuzzerPlayback.index = 0;
        gBuzzerPlayback.startMs = nowMs;
      } else {
        stopBuzzerPlayback();
        return;
      }
    }

    if (!gBuzzerPlayback.active) {
      return;
    }

    const BuzzerPwmStep &step = gBuzzerPlayback.steps[gBuzzerPlayback.index++];
    buzzerSetTone(step.freq_hz, gBuzzerPlayback.duty);

    if (step.duration_ms > 0U) {
      const uint32_t durationMs = gBuzzerPlayback.tempoScale
                                      ? scaleDurationForTempo(step.duration_ms)
                                      : step.duration_ms;
      gBuzzerPlayback.deadlineMs = nowMs + durationMs;
      return;
    }
  }
}

void updateBootBuzzer() {
  if (gBootBuzzerStartRequested) {
    gBootBuzzerStartRequested = false;
    if (MasterBusiness::getBootSoundEnabled()) {
      startBootBuzzer();
    }
  }

  UiMonitorSnapshot snap{};
  const bool hasSnap = MasterBusiness::getMonitorSnapshot(snap);
  const uint32_t linkInitSuccessCount =
      MasterBusiness::getLinkInitSuccessCount();
  const bool errorState =
      hasSnap && ((snap.errFlags & CCTRL_ERROR_TIMEOUT) != 0U ||
                  snap.disconnectMode != 0U);
  const bool wristLimitState = hasSnap && snap.wristLimitActive != 0U;
  const uint8_t keyFlags = hasSnap ? snap.keyAudioFlags : 0;
  const uint8_t keyPressMask = (uint8_t)(keyFlags & (uint8_t)~gLastKeyFlags);
  const uint8_t doubleClickFlags = hasSnap ? snap.keyDoubleClickFlags : 0;
  const uint8_t doubleClickMask =
      (uint8_t)(doubleClickFlags & (uint8_t)~gLastDoubleClickFlags);
  const uint8_t handleButtons = hasSnap ? snap.handleButtons : 0;
  const uint8_t handlePressMask =
      (uint8_t)(handleButtons & (uint8_t)~gLastHandleButtons);
  const bool s4Released =
      (gLastHandleButtons & 0x08u) != 0U && (handleButtons & 0x08u) == 0U;
  const bool anyKeyPressed = keyPressMask != 0U || handlePressMask != 0U;

  if (isBootBuzzerActive() && anyKeyPressed) {
    stopBuzzerPlayback();
  }

  if (linkInitSuccessCount != gLastLinkInitSuccessCount) {
    startLinkOkBuzzer();
    gLastLinkInitSuccessCount = linkInitSuccessCount;
  }

  if (doubleClickMask != 0U) {
    requestPromptPlayback(kKeyDoubleClickPattern, kKeyDoubleClickPatternCount,
                          BUZZER_PWM_DUTY_KEY_PRESS, kKeyPressGuardMs, false,
                          BUZZER_SOUND_KEY_PRESS);
  } else if (s4Released) {
    startKeyReleaseBuzzer();
  } else if (anyKeyPressed) {
    startKeyPressBuzzer();
  }
  if (!hasSnap) {
    gLastKeyFlags = 0;
    gLastDoubleClickFlags = 0;
    gLastHandleButtons = 0;
  } else {
    gLastKeyFlags = keyFlags;
    gLastDoubleClickFlags = doubleClickFlags;
    gLastHandleButtons = handleButtons;
  }

  if (!MasterBusiness::getBootSoundEnabled() && isBootBuzzerActive()) {
    stopBuzzerPlayback();
  }

  if (!errorState && gBuzzerPlayback.active &&
      gBuzzerPlayback.soundKind == BUZZER_SOUND_ERROR) {
    stopBuzzerPlayback();
  }
  if (!wristLimitState && gBuzzerPlayback.active &&
      gBuzzerPlayback.soundKind == BUZZER_SOUND_WRIST_LIMIT) {
    stopBuzzerPlayback();
  }

  if (errorState && !gLastErrorState) {
    startErrorBuzzer();
  }
  gLastErrorState = errorState;

  const uint32_t nowMs = millis();
  if (gBuzzerPlayback.active) {
    if ((int32_t)(nowMs - gBuzzerPlayback.startMs) >=
        (int32_t)gBuzzerPlayback.guardMs) {
      stopBuzzerPlayback();
    } else if ((int32_t)(nowMs - gBuzzerPlayback.deadlineMs) >= 0) {
      advanceBuzzerPlayback(nowMs, errorState);
    }
  }

  if (!gBuzzerPlayback.active && errorState && !isBootBuzzerActive()) {
    startErrorBuzzer();
  } else if (!gBuzzerPlayback.active && wristLimitState &&
             !isBootBuzzerActive()) {
    startWristLimitBuzzer();
  }
}

void startBootBuzzer() {
  stopBuzzerPlayback();

  if (!MasterBusiness::getBootSoundEnabled()) {
    return;
  }
  startBuzzerPlayback(kIceBootMelody, kIceBootMelodyCount, BUZZER_PWM_DUTY_BOOT,
                      scaleDurationForTempo(kIceBootMelodyTotalMs) +
                          BOOT_MELODY_STOP_GUARD_MS,
                      true, false, BUZZER_SOUND_BOOT);
  if (gBuzzerPlayback.active) {
    advanceBuzzerPlayback(gBuzzerPlayback.startMs, false);
  }
}

void startErrorBuzzer() {
  requestPromptPlayback(kErrorBeepPattern, kErrorBeepPatternCount,
                        BUZZER_PWM_DUTY_ERROR, kErrorBeepGuardMs, true,
                        BUZZER_SOUND_ERROR);
}

void startLinkOkBuzzer() {
  requestPromptPlayback(kLinkOkPattern, kLinkOkPatternCount,
                        BUZZER_PWM_DUTY_LINK_OK, kLinkOkGuardMs, false,
                        BUZZER_SOUND_LINK_OK);
}

void startKeyPressBuzzer() {
  requestPromptPlayback(kKeyPressPattern, kKeyPressPatternCount,
                        BUZZER_PWM_DUTY_KEY_PRESS, kKeyPressGuardMs, false,
                        BUZZER_SOUND_KEY_PRESS);
}

void startKeyReleaseBuzzer() {
  requestPromptPlayback(kKeyReleasePattern, kKeyReleasePatternCount,
                        BUZZER_PWM_DUTY_KEY_PRESS, kKeyReleaseGuardMs, false,
                        BUZZER_SOUND_KEY_RELEASE);
}

void startWristLimitBuzzer() {
  requestPromptPlayback(kWristLimitPattern, kWristLimitPatternCount,
                        BUZZER_PWM_DUTY_WRIST_LIMIT, kWristLimitGuardMs, false,
                        BUZZER_SOUND_WRIST_LIMIT);
}

} // namespace

namespace TileMenuAudio {

void playNavigateBackward() {
  requestPromptPlayback(kTileNavBackwardPattern, kTileNavBackwardPatternCount,
                        BUZZER_PWM_DUTY_KEY_PRESS, kTileNavBackwardGuardMs,
                        false, BUZZER_SOUND_TILE_NAV_BACKWARD);
}

void playNavigateForward() {
  requestPromptPlayback(kTileNavForwardPattern, kTileNavForwardPatternCount,
                        BUZZER_PWM_DUTY_KEY_PRESS, kTileNavForwardGuardMs,
                        false, BUZZER_SOUND_TILE_NAV_FORWARD);
}

void playConfirm() {
  requestPromptPlayback(kTileConfirmPattern, kTileConfirmPatternCount,
                        BUZZER_PWM_DUTY_LINK_OK, kTileConfirmGuardMs, false,
                        BUZZER_SOUND_TILE_CONFIRM);
}

} // namespace TileMenuAudio

namespace {

bool gStatLedOn = false;
unsigned long gLastBlinkMs = 0;
TaskHandle_t gUiTaskHandle = nullptr;
TaskHandle_t gBusinessTaskHandle = nullptr;

void uiTask(void *) {
  WouoUiLiteGeneral::begin();

  for (;;) {
    updateBootBuzzer();

    UiMonitorSnapshot snap{};
    bool hasSnap = MasterBusiness::getMonitorSnapshot(snap);
    digitalWrite(PIN_LED_ERR,
                 (hasSnap && (snap.errFlags != 0U || snap.disconnectMode))
                     ? HIGH
                     : LOW);

    WouoUiLiteGeneral::tick();

    const unsigned long nowMs = millis();
    if ((nowMs - gLastBlinkMs) >= 400UL) {
      gLastBlinkMs = nowMs;
      gStatLedOn = !gStatLedOn;
      digitalWrite(PIN_LED_STAT, gStatLedOn ? HIGH : LOW);
    }

    vTaskDelay(1);
  }
}

void businessTask(void *) {
  MasterBusiness::setup();
  if (MasterBusiness::getBootSoundEnabled()) {
    gBootBuzzerStartRequested = true;
  }

  for (;;) {
    MasterBusiness::loop();
    taskYIELD();
  }
}
} // namespace

void setup() {
  pinMode(PIN_LED_ERR, OUTPUT);
  pinMode(PIN_LED_STAT, OUTPUT);

  digitalWrite(PIN_LED_ERR, HIGH);
  digitalWrite(PIN_LED_STAT, HIGH);
  delay(400);
  digitalWrite(PIN_LED_ERR, LOW);
  digitalWrite(PIN_LED_STAT, LOW);

  xTaskCreatePinnedToCore(uiTask, "ui_core0", 8192, nullptr, 2, &gUiTaskHandle,
                          0);
  xTaskCreatePinnedToCore(businessTask, "biz_core1", 12288, nullptr, 3,
                          &gBusinessTaskHandle, 1);
}

void loop() { delay(1000); }
