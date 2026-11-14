#pragma once

#if defined(UNIT_TEST)

#include "Hal/EscDriver.hpp"
#include <cstddef>
#include <cstdint>

namespace Hal::Detail {

// host fake, records attached pins only
class EscDriverImpl
{
public:
  EscDriverImpl()
  {
    reset();
  }

  bool begin(const EscConfig& conf)
  {
    reset();
    return true;
  }

  void end() {}

  bool attach(size_t channel, int pin, int pulse)
  {
    _pins[channel] = pin;
    return true;
  }

  void write(size_t channel, int pulse) {}

  void apply() {}

  int pin(size_t channel) const
  {
    return _pins[channel];
  }

  uint32_t telemetry(size_t channel) const
  {
    return 0;
  }

private:
  void reset()
  {
    for (auto& p : _pins)
    {
      p = -1;
    }
  }

  int _pins[EscDriver::CHANNEL_COUNT];
};

} // namespace Hal::Detail

#endif
