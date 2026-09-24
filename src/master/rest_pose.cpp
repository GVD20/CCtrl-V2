#include "rest_pose.h"

namespace RestPose {

uint16_t circularDistance(uint16_t a, uint16_t b) {
  const uint16_t forward = (uint16_t)((a - b) & 0x0FFFu);
  return forward > 2048u ? (uint16_t)(4096u - forward) : forward;
}

bool inside(const uint16_t raw[6], const uint16_t reference[6],
            const uint16_t tolerance[6]) {
  for (uint8_t i = 0; i < 6; ++i)
    if (circularDistance(raw[i], reference[i]) > tolerance[i]) return false;
  return true;
}

bool update(State &state, const uint16_t raw[6], const uint16_t reference[6],
            bool sensorsHealthy, uint32_t nowMs) {
  const bool target = state.in_region
                          ? !(sensorsHealthy &&
                              inside(raw, reference, kExitToleranceCounts))
                          : sensorsHealthy &&
                                inside(raw, reference, kEnterToleranceCounts);
  const bool targetState = state.in_region ? !target : target;

  if (targetState == state.in_region) {
    state.candidate_active = false;
    return false;
  }
  if (!state.candidate_active || state.candidate_target != targetState) {
    state.candidate_active = true;
    state.candidate_target = targetState;
    state.candidate_since_ms = nowMs;
    return false;
  }
  if ((uint32_t)(nowMs - state.candidate_since_ms) < kStableMs) return false;

  state.in_region = targetState;
  state.candidate_active = false;
  return true;
}

} // namespace RestPose
