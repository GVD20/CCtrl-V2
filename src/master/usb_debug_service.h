#pragma once

#include <stdint.h>

namespace UsbDebugService {

bool requestStart();
bool enabled();
bool connected();
void loop(uint32_t nowMs);

} // namespace UsbDebugService
