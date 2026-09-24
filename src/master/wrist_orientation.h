#pragma once

#include <stdint.h>

namespace WristOrientation {

struct State {
  float alpha = 0.0f;
  float beta = 0.0f;
  float gamma = 0.0f;
  bool initialized = false;
  bool singular = false;
  bool recovering = false;
  bool soft_limited = false;
  uint8_t reset_frames = 30;
};

void reset(State &state);

// Right-handed intrinsic active rotations:
// Rx(-A4) * Rz(A5) * Ry(-A6) = Rz(alpha) * Rx(beta) * Rz(gamma).
// The USB debugger presents this controller input mapping without conversion.
// Input and output use signed 4096-count turns. Call exactly once per emitted
// 30 Hz host frame so recoveryStep has the same meaning as the web reference.
void xzyToZxz(const int16_t xzy[3], int16_t zxz[3], State &state);

} // namespace WristOrientation
