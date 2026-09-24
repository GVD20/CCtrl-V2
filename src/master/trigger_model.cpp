#include "trigger_model.h"

#include <stddef.h>

namespace {

struct Point3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

// Median trajectory from seven press/release captures.
// Each interval is 1/16 travel.
constexpr Point3 kCurve[] = {
    {900, -323, 6402},      {1378, -467, 7693},
    {1768, -624, 8348},    {2479, -848, 9743},
    {3376, -1085, 11268},  {4333, -1398, 12874},
    {5803, -1794, 14976},  {8264, -2345, 17878},
    {12261, -3175, 21724}, {16325, -4051, 26306},
    {19695, -4788, 28167}, {24083, -5290, 30667},
    {28163, -6300, 32297}, {31499, -8188, 32767},
    {32592, -9902, 32767}, {32767, -12634, 32624},
    {32592, -15130, 32624},
};
constexpr size_t kPointCount = sizeof(kCurve) / sizeof(kCurve[0]);
constexpr int32_t kFractionScale = 32768;

} // namespace

namespace TriggerModel {

uint16_t estimate(int16_t x, int16_t y, int16_t z) {
  uint64_t bestDistance = UINT64_MAX;
  uint32_t bestPosition = 0;

  for (size_t i = 0; i + 1 < kPointCount; ++i) {
    const int32_t dx = (int32_t)kCurve[i + 1].x - kCurve[i].x;
    const int32_t dy = (int32_t)kCurve[i + 1].y - kCurve[i].y;
    const int32_t dz = (int32_t)kCurve[i + 1].z - kCurve[i].z;
    const int64_t lengthSquared =
        (int64_t)dx * dx + (int64_t)dy * dy + (int64_t)dz * dz;
    const int64_t dot = (int64_t)((int32_t)x - kCurve[i].x) * dx +
                        (int64_t)((int32_t)y - kCurve[i].y) * dy +
                        (int64_t)((int32_t)z - kCurve[i].z) * dz;
    int32_t fraction = (int32_t)((dot * kFractionScale) / lengthSquared);
    if (fraction < 0) fraction = 0;
    if (fraction > kFractionScale) fraction = kFractionScale;

    const int32_t px = kCurve[i].x +
                       (int32_t)(((int64_t)dx * fraction) / kFractionScale);
    const int32_t py = kCurve[i].y +
                       (int32_t)(((int64_t)dy * fraction) / kFractionScale);
    const int32_t pz = kCurve[i].z +
                       (int32_t)(((int64_t)dz * fraction) / kFractionScale);
    const int32_t rx = (int32_t)x - px;
    const int32_t ry = (int32_t)y - py;
    const int32_t rz = (int32_t)z - pz;
    const uint64_t distance = (uint64_t)((int64_t)rx * rx +
                                         (int64_t)ry * ry +
                                         (int64_t)rz * rz);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestPosition = (uint32_t)i * kFractionScale + (uint32_t)fraction;
    }
  }

  const uint32_t fullScale = (uint32_t)(kPointCount - 1) * kFractionScale;
  return (uint16_t)((bestPosition * 4095u + fullScale / 2u) / fullScale);
}

} // namespace TriggerModel
