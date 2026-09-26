#if defined(ARCH_RP2040)

#include "Hal/HwTimer.hpp"
#include <hardware/timer.h>
#include <pico/time.h>

namespace Espfc::Hal {

// Hardware alarms are one shot, the handler rearms them. Data needed in interrupt context is kept
// in this table, so that the handler does not have to touch the HwTimer instance.
struct AlarmSlot
{
  HwTimer::Callback callback;
  void* arg;
  uint32_t interval;
  uint64_t next;
};

static AlarmSlot alarmSlots[NUM_ALARMS] = {};

static inline void __not_in_flash_func(rearm)(uint num, AlarmSlot& slot, uint64_t next)
{
  while (hardware_alarm_set_target(num, from_us_since_boot(next)))
  {
    next += slot.interval;
  }
  slot.next = next;
}

static void __not_in_flash_func(alarmHandler)(uint num)
{
  AlarmSlot& slot = alarmSlots[num];
  if (!slot.callback) return;

  const uint64_t now = time_us_64();
  uint64_t next = slot.next + slot.interval;
  if (next <= now) next = now + slot.interval;
  rearm(num, slot, next);

  // the context switch request is only meaningful on FreeRTOS targets
  slot.callback(slot.arg);
}

// The alarm interrupt is installed on the calling core, begin() has to be called from the core
// that is supposed to be interrupted. The id passed to the constructor is ignored, _id holds the
// dynamically claimed alarm number instead.
bool HwTimer::begin(uint32_t intervalUs, Callback callback, void* arg)
{
  if (_running || !callback || !intervalUs) return false;

  const int num = hardware_alarm_claim_unused(false);
  if (num < 0) return false;

  _callback = callback;
  _arg = arg;
  _interval = intervalUs;
  _id = static_cast<uint8_t>(num);

  AlarmSlot& slot = alarmSlots[num];
  slot.callback = _callback;
  slot.arg = _arg;
  slot.interval = _interval;

  hardware_alarm_set_callback(num, alarmHandler);
  rearm(num, slot, time_us_64() + _interval);

  _running = true;
  return true;
}

void HwTimer::end()
{
  if (!_running) return;

  hardware_alarm_cancel(_id);
  hardware_alarm_set_callback(_id, nullptr);
  hardware_alarm_unclaim(_id);
  alarmSlots[_id] = AlarmSlot{};

  _running = false;
}

} // namespace Espfc::Hal

#endif
