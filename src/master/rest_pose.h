#pragma once

#include <stdint.h>

namespace RestPose {

constexpr uint16_t kEnterToleranceCounts[6] = {342, 23, 23, 23, 342, 342};
constexpr uint16_t kExitToleranceCounts[6] = {399, 35, 35, 35, 399, 399};
constexpr uint32_t kStableMs = 500;

struct State {
  bool in_region = false;
  bool candidate_active = false;
  bool candidate_target = false;
  uint32_t candidate_since_ms = 0;
};

uint16_t circularDistance(uint16_t a, uint16_t b);
bool inside(const uint16_t raw[6], const uint16_t reference[6],
            const uint16_t tolerance[6]);
bool update(State &state, const uint16_t raw[6], const uint16_t reference[6],
            bool sensorsHealthy, uint32_t nowMs);

} // namespace RestPose
