#pragma once

#include "Hal/FastCode.hpp"
#include "Hal/Gpio.hpp"
#include "Hal/Time.hpp"
#include <cstddef>
#include <cstdint>

namespace Espfc::Hal::Detail {

struct CaptureSlot
{
  Gpio::CaptureHandler handler = nullptr;
  void* arg = nullptr;
  uint32_t last = 0;
  int16_t pin = -1;
  bool seen = false;
};

constexpr size_t CAPTURE_SLOTS = 6;

inline CaptureSlot captureSlots[CAPTURE_SLOTS];

inline CaptureSlot* captureFind(uint8_t pin)
{
  for (auto& s : captureSlots)
  {
    if (s.pin == pin) return &s;
  }
  return nullptr;
}

inline CaptureSlot* captureAcquire(uint8_t pin, Gpio::CaptureHandler handler, void* arg)
{
  if (!handler) return nullptr;
  if (auto* e = captureFind(pin)) e->pin = -1;
  for (auto& s : captureSlots)
  {
    if (s.pin != -1) continue;
    s.handler = handler;
    s.arg = arg;
    s.seen = false;
    s.pin = pin;
    return &s;
  }
  return nullptr;
}

static void ISR_CODE_ATTR captureTrampoline(void* arg)
{
  auto* s = static_cast<CaptureSlot*>(arg);
  const uint32_t now = static_cast<uint32_t>(micros());
  if (s->seen) s->handler(s->arg, now - s->last);
  s->last = now;
  s->seen = true;
}

inline bool captureAttachFallback(uint8_t pin, Gpio::CaptureHandler handler, void* arg, Gpio::InterruptMode mode)
{
  auto* s = captureAcquire(pin, handler, arg);
  if (!s) return false;
  Gpio::attachInterrupt(pin, captureTrampoline, s, mode);
  return true;
}

inline void captureDetachFallback(uint8_t pin)
{
  auto* s = captureFind(pin);
  if (!s) return;
  Gpio::detachInterrupt(pin);
  s->pin = -1;
}

} // namespace Espfc::Hal::Detail
