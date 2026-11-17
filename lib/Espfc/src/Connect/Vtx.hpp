#pragma once

#include "ModelConfig.hpp"
#include <algorithm>

namespace Espfc::Connect {

enum VtxDeviceType
{
  VTXDEV_UNSUPPORTED = 0, // reserved for MSP
  VTXDEV_RTC6705 = 1,
  // 2 reserved
  VTXDEV_SMARTAUDIO = 3,
  VTXDEV_TRAMP = 4,
  VTXDEV_MSP = 5,
  VTXDEV_UNKNOWN = 0xFF,
};

constexpr int VTX_MIN_FREQUENCY_MHZ = 5000;
constexpr int VTX_MAX_FREQUENCY_MHZ = 5999;

// Frequency in MHz requested directly, 0 when band and channel should be used.
inline uint16_t vtxDirectFrequency(const VtxConfig& c)
{
  const int f = (c.pitMode && c.pitModeFrequency) ? c.pitModeFrequency : (c.band == 0 ? c.frequency : 0);
  return f > 0 ? std::clamp(f, VTX_MIN_FREQUENCY_MHZ, VTX_MAX_FREQUENCY_MHZ) : 0;
}

} // namespace Espfc::Connect
