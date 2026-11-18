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

constexpr uint8_t VTX_BAND_COUNT = 5;
constexpr uint8_t VTX_CHANNEL_COUNT = 8;
constexpr int VTX_MIN_FREQUENCY_MHZ = 5000;
constexpr int VTX_MAX_FREQUENCY_MHZ = 5999;

// Zero means unset.
inline bool vtxIsValidFrequency(int f)
{
  return f == 0 || (f >= VTX_MIN_FREQUENCY_MHZ && f <= VTX_MAX_FREQUENCY_MHZ);
}

inline uint16_t vtxClampFrequency(int f)
{
  return f > 0 ? std::clamp(f, VTX_MIN_FREQUENCY_MHZ, VTX_MAX_FREQUENCY_MHZ) : 0;
}

// Frequency in MHz configured directly (band 0), 0 when band and channel should be used.
inline uint16_t vtxBaseFrequency(const VtxConfig& c)
{
  return c.band == 0 ? vtxClampFrequency(c.frequency) : 0;
}

// Like vtxBaseFrequency, but tuned to pit mode frequency when pit mode is active.
inline uint16_t vtxDirectFrequency(const VtxConfig& c)
{
  return (c.pitMode && c.pitModeFrequency) ? vtxClampFrequency(c.pitModeFrequency) : vtxBaseFrequency(c);
}

} // namespace Espfc::Connect
