#pragma once

#include <stdint.h>

namespace TriggerModel {

// Maps the captured TMAG5273 XYZ trajectory to 0..4095 trigger travel.
// The first and last calibration points sit inside the endpoint plateaus, so
// projection naturally clamps a small region at each physical limit.
uint16_t estimate(int16_t x, int16_t y, int16_t z);

} // namespace TriggerModel
