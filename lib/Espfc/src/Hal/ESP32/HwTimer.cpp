#if defined(ESP32)

#include "Hal/HwTimer.hpp"
#include <driver/timer.h>
#include <soc/soc_caps.h>

namespace Espfc::Hal {

static inline timer_group_t timerGroup(uint8_t id)
{
  return static_cast<timer_group_t>(id / SOC_TIMER_GROUP_TIMERS_PER_GROUP);
}

static inline timer_idx_t timerIdx(uint8_t id)
{
  return static_cast<timer_idx_t>(id % SOC_TIMER_GROUP_TIMERS_PER_GROUP);
}

bool HwTimer::begin(uint32_t intervalUs, Callback callback, void* arg)
{
  if (_running || !callback || !intervalUs || _id >= SOC_TIMER_GROUP_TOTAL_TIMERS) return false;

  _callback = callback;
  _arg = arg;
  _interval = intervalUs;

  const timer_group_t group = timerGroup(_id);
  const timer_idx_t idx = timerIdx(_id);

  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .divider = 80, // 1MHz tick from 80MHz APB clock
  };
  if (timer_init(group, idx, &config) != ESP_OK) return false;
  timer_set_counter_value(group, idx, 0);
  timer_set_alarm_value(group, idx, _interval);
  timer_isr_callback_add(group, idx, _callback, _arg, ESP_INTR_FLAG_IRAM);
  timer_enable_intr(group, idx);
  timer_start(group, idx);

  _running = true;
  return true;
}

void HwTimer::end()
{
  if (!_running) return;

  const timer_group_t group = timerGroup(_id);
  const timer_idx_t idx = timerIdx(_id);

  timer_pause(group, idx);
  timer_disable_intr(group, idx);
  timer_isr_callback_remove(group, idx);
  timer_deinit(group, idx);

  _running = false;
}

} // namespace Espfc::Hal

#endif
