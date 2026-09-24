#pragma once

#include <stdint.h>

namespace JoystickModel {

uint16_t calibrateX(uint16_t raw);
uint16_t calibrateY(uint16_t raw);

} // namespace JoystickModel
