#include "joystick_model.h"

namespace {

struct AxisCalibration {
  uint16_t minimum;
  uint16_t center;
  uint16_t maximum;
  uint16_t deadZone;
};

// Fitted from 900 joystick samples; see docs/JOYSTICK_CALIBRATION_ZHCN.md.
constexpr AxisCalibration kX = {805, 2010, 3226, 50};
constexpr AxisCalibration kY = {857, 1985, 3150, 50};

uint16_t calibrate(uint16_t raw, const AxisCalibration &axis) {
  const uint16_t lowerCenter = axis.center - axis.deadZone;
  const uint16_t upperCenter = axis.center + axis.deadZone;
  if (raw <= axis.minimum) return 0;
  if (raw >= axis.maximum) return 4095;
  if (raw >= lowerCenter && raw <= upperCenter) return 2048;

  if (raw < lowerCenter) {
    const uint32_t numerator = (uint32_t)(raw - axis.minimum) * 2047u;
    const uint16_t span = lowerCenter - axis.minimum;
    return (uint16_t)((numerator + span / 2u) / span);
  }

  const uint32_t numerator = (uint32_t)(raw - upperCenter) * 2047u;
  const uint16_t span = axis.maximum - upperCenter;
  return (uint16_t)(2048u + (numerator + span / 2u) / span);
}

} // namespace

namespace JoystickModel {

uint16_t calibrateX(uint16_t raw) { return calibrate(raw, kX); }
uint16_t calibrateY(uint16_t raw) {
  const uint16_t value = calibrate(raw, kY);
  if (value == 0u) return 4095u;
  if (value == 4095u) return 0u;
  return (uint16_t)(4096u - value);
}

} // namespace JoystickModel
