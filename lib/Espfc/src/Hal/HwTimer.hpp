#pragma once

#include <cstdint>

namespace Espfc::Hal {

// Periodic hardware timer, the callback runs in interrupt context and must be placed in fast memory.
class HwTimer
{
public:
  // Return true when a higher priority task was woken and a context switch is required on ISR exit.
  using Callback = bool (*)(void* arg);

  constexpr explicit HwTimer(uint8_t id = 0): _callback(nullptr), _arg(nullptr), _interval(0), _id(id), _running(false)
  {
  }

  bool begin(uint32_t intervalUs, Callback callback, void* arg = nullptr);
  void end();

  bool isRunning() const
  {
    return _running;
  }

private:
  Callback _callback;
  void* _arg;
  uint32_t _interval;
  uint8_t _id;
  bool _running;
};

} // namespace Espfc::Hal
