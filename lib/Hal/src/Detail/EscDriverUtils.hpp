#pragma once

#include "Hal/FastCode.hpp"

namespace Hal::Detail {

// Arduino map() is placed in flash, which is unsafe in isr
static inline long ISR_CODE_ATTR escMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  const long in_range = in_max - in_min;
  if (in_range == 0) return out_min;
  const long out_range = out_max - out_min;
  const long delta = x - in_min;
  return (delta * out_range) / in_range + out_min;
}

} // namespace Hal::Detail
