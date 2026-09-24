#include "wrist_orientation.h"

#include <math.h>

namespace WristOrientation {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kRadiansPerCount = kTwoPi / 4096.0f;
constexpr float kSingularEnter = 3.0f * kPi / 180.0f;
constexpr float kSingularExit = 5.0f * kPi / 180.0f;
constexpr float kRecoveryStep = 9.0f * kPi / 180.0f;
constexpr float kRecoveryEpsilon = 1.0e-9f;
constexpr float kMiddleSoftLimitLower = -87.0f * kPi / 180.0f;
constexpr float kMiddleSoftLimitUpper = 112.0f * kPi / 180.0f;
constexpr float kMiddleHardLimitLower = -90.0f * kPi / 180.0f;
constexpr float kMiddleHardLimitUpper = 115.0f * kPi / 180.0f;

float wrapNear(float value, float reference) {
  return value + roundf((reference - value) / kTwoPi) * kTwoPi;
}

float moveNear(float current, float target) {
  const float delta = wrapNear(target, current) - current;
  if (delta > kRecoveryStep) return current + kRecoveryStep;
  if (delta < -kRecoveryStep) return current - kRecoveryStep;
  return current + delta;
}

bool recoveryPending(float current, float target) {
  return fabsf(wrapNear(target, current) - current) > kRecoveryEpsilon;
}

float softLimitMiddleAxis(float radians, bool &limited) {
  if (radians < kMiddleSoftLimitLower) {
    limited = true;
    const float span = kMiddleSoftLimitLower - kMiddleHardLimitLower;
    return kMiddleSoftLimitLower -
           span * tanhf((kMiddleSoftLimitLower - radians) / span);
  }
  if (radians > kMiddleSoftLimitUpper) {
    limited = true;
    const float span = kMiddleHardLimitUpper - kMiddleSoftLimitUpper;
    return kMiddleSoftLimitUpper +
           span * tanhf((radians - kMiddleSoftLimitUpper) / span);
  }
  limited = false;
  return radians;
}

int16_t radiansToCounts(float radians, bool middleAxis = false) {
  int32_t counts = (int32_t)lroundf(radians / kRadiansPerCount);
  // +pi and -pi share the same signed cyclic code. For the ZXZ middle axis,
  // keep a positive approach to +pi on the positive side of the range.
  if (middleAxis && counts > 0 && (counts & 0x0FFF) == 2048) return 2047;
  counts %= 4096;
  if (counts < -2048) counts += 4096;
  if (counts > 2047) counts -= 4096;
  return (int16_t)counts;
}

float changeScore(float alpha, float beta, float gamma, const State &state) {
  const float da = alpha - state.alpha;
  const float db = beta - state.beta;
  const float dg = gamma - state.gamma;
  return da * da + 2.0f * db * db + dg * dg;
}

} // namespace

void reset(State &state) { state = State{}; }

void xzyToZxz(const int16_t xzy[3], int16_t zxz[3], State &state) {
  // Match the web controller mapping exactly: the cards and XZY wire values
  // remain raw A4/A5/A6, while the simulated orientation reverses A4/X and
  // A6/Y before decomposition.
  const float x = -xzy[0] * kRadiansPerCount;
  const float z = xzy[1] * kRadiansPerCount;
  const float y = -xzy[2] * kRadiansPerCount;
  const float cx = cosf(x), sx = sinf(x);
  const float cz = cosf(z), sz = sinf(z);
  const float cy = cosf(y), sy = sinf(y);

  // R = Rx(x) * Rz(z) * Ry(y). Only the entries required by ZXZ extraction
  // and singular-axis recovery are formed.
  const float r00 = cz * cy;
  const float r10 = cx * sz * cy + sx * sy;
  const float r02 = cz * sy;
  const float r12 = cx * sz * sy - sx * cy;
  const float r20 = sx * sz * cy - cx * sy;
  const float r21 = sx * cz;
  const float r22 = sx * sz * sy + cx * cy;

  const float clampedR22 = fmaxf(-1.0f, fminf(1.0f, r22));
  const float principalBeta = acosf(clampedR22);
  const float singularDistance =
      fminf(principalBeta, kPi - principalBeta);
  float alpha;
  float beta;
  float gamma;

  if (!state.initialized) {
    state.singular = singularDistance <= kSingularEnter;
    state.recovering = false;
    if (singularDistance < kSingularEnter) {
      alpha = atan2f(r10, r00);
      beta = principalBeta;
      gamma = 0.0f;
    } else {
      alpha = atan2f(r02, -r12);
      beta = principalBeta;
      gamma = atan2f(r20, r21);
    }
    state.initialized = true;
  } else {
    if (state.singular) {
      if (singularDistance >= kSingularExit) {
        state.singular = false;
        state.recovering = true;
      }
    } else if (singularDistance <= kSingularEnter) {
      state.singular = true;
      state.recovering = false;
    }

    if (state.singular && principalBeta < kPi - principalBeta) {
      // At beta=0 only alpha+gamma is observable. Split its small change
      // evenly so neither wrist Z axis jumps through the singularity.
      const float sum =
          wrapNear(atan2f(r10, r00), state.alpha + state.gamma);
      const float halfDelta = 0.5f * (sum - state.alpha - state.gamma);
      alpha = state.alpha + halfDelta;
      gamma = state.gamma + halfDelta;
      beta = state.beta < 0.0f ? -principalBeta : principalBeta;
    } else if (state.singular) {
      // At beta=pi only alpha-gamma is observable.
      const float difference =
          wrapNear(atan2f(r10, r00), state.alpha - state.gamma);
      const float halfDelta =
          0.5f * (difference - state.alpha + state.gamma);
      alpha = state.alpha + halfDelta;
      gamma = state.gamma - halfDelta;
      const float positive = wrapNear(principalBeta, state.beta);
      const float negative = wrapNear(-principalBeta, state.beta);
      beta = fabsf(positive - state.beta) <= fabsf(negative - state.beta)
                 ? positive
                 : negative;
    } else {
      // Two exact ZXZ branches describe the same rotation. Select and unwrap
      // the one closest to the preceding output.
      const float baseAlpha = atan2f(r02, -r12);
      const float baseGamma = atan2f(r20, r21);
      const float a1 = wrapNear(baseAlpha, state.alpha);
      const float b1 = wrapNear(principalBeta, state.beta);
      const float g1 = wrapNear(baseGamma, state.gamma);
      const float a2 = wrapNear(baseAlpha + kPi, state.alpha);
      const float b2 = wrapNear(-principalBeta, state.beta);
      const float g2 = wrapNear(baseGamma + kPi, state.gamma);
      if (changeScore(a1, b1, g1, state) <=
          changeScore(a2, b2, g2, state)) {
        alpha = a1;
        beta = b1;
        gamma = g1;
      } else {
        alpha = a2;
        beta = b2;
        gamma = g2;
      }
    }

    if (state.recovering) {
      const float targetAlpha = alpha;
      const float targetBeta = beta;
      const float targetGamma = gamma;
      alpha = moveNear(state.alpha, targetAlpha);
      beta = moveNear(state.beta, targetBeta);
      gamma = moveNear(state.gamma, targetGamma);
      state.recovering = recoveryPending(alpha, targetAlpha) ||
                         recoveryPending(beta, targetBeta) ||
                         recoveryPending(gamma, targetGamma);
    }
  }

  state.alpha = alpha;
  state.beta = beta;
  state.gamma = gamma;
  if (state.reset_frames > 0) --state.reset_frames;
  zxz[0] = radiansToCounts(alpha);
  zxz[1] = radiansToCounts(softLimitMiddleAxis(beta, state.soft_limited), true);
  zxz[2] = radiansToCounts(gamma);
}

} // namespace WristOrientation
