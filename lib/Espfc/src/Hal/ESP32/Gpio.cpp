#if defined(ESP32)

#include "Hal/Gpio.hpp"
#include "Hal/Detail/GpioCapture.hpp"
#include "Hal/FastCode.hpp"
#include "hal/gpio_ll.h"
#include "soc/soc_caps.h"
#include <Arduino.h>
#if SOC_MCPWM_SUPPORTED
#include "driver/mcpwm.h"
#endif

namespace Espfc::Hal {

void FAST_CODE_ATTR Gpio::digitalWrite(uint8_t pin, Gpio::PinStatus val)
{
  //::gpio_set_level((gpio_num_t)pin, val);
  gpio_ll_set_level(&GPIO, (gpio_num_t)pin, val);
}

Gpio::PinStatus FAST_CODE_ATTR Gpio::digitalRead(uint8_t pin)
{
  // return ::gpio_get_level((gpio_num_t)pin);
  return ::gpio_ll_get_level(&GPIO, (gpio_num_t)pin) ? Gpio::PinStatus::High : Gpio::PinStatus::Low;
}

void Gpio::pinMode(uint8_t pin, Gpio::PinMode mode)
{
  switch (mode)
  {
    case Gpio::Input:
      ::pinMode(pin, INPUT);
      break;
    case Gpio::InputPullup:
      ::pinMode(pin, INPUT_PULLUP);
      break;
    case Gpio::Output:
      ::pinMode(pin, OUTPUT);
      break;
  }
}

static_assert(Gpio::InterruptMode::Rising == RISING);
static_assert(Gpio::InterruptMode::Falling == FALLING);
static_assert(Gpio::InterruptMode::Change == CHANGE);

void Gpio::attachInterrupt(uint8_t pin, InterruptHandler handler, void* arg, InterruptMode mode)
{
  ::attachInterruptArg(pin, handler, arg, mode);
}

void Gpio::detachInterrupt(uint8_t pin)
{
  ::detachInterrupt(pin);
}

#if SOC_MCPWM_SUPPORTED

static constexpr uint32_t MCPWM_TICKS_PER_US = APB_CLK_FREQ / 1000000UL;
static constexpr uint32_t MCPWM_TICKS_ROUNDING = MCPWM_TICKS_PER_US / 2;

static bool IRAM_ATTR mcpwmCaptureIsr(mcpwm_unit_t, mcpwm_capture_channel_id_t, const cap_event_data_t* edata,
                                      void* arg)
{
  auto* s = static_cast<Detail::CaptureSlot*>(arg);
  const uint32_t now = edata->cap_value;
  if (s->seen) s->handler(s->arg, (now - s->last + MCPWM_TICKS_ROUNDING) / MCPWM_TICKS_PER_US);
  s->last = now;
  s->seen = true;
  return false;
}

bool Gpio::attachCaptureInterrupt(uint8_t pin, CaptureHandler handler, void* arg, InterruptMode mode)
{
  if (mode == Change) return false;

  if (Detail::captureFind(pin)) detachCaptureInterrupt(pin);
  auto* s = Detail::captureAcquire(pin, handler, arg);
  if (!s) return false;

  const size_t idx = s - Detail::captureSlots;
  const auto unit = static_cast<mcpwm_unit_t>(idx / SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER);
  const auto ch = static_cast<mcpwm_capture_channel_id_t>(idx % SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER);

  pinMode(pin, Input);
  mcpwm_capture_config_t cfg = {};
  cfg.cap_edge = mode == Rising ? MCPWM_POS_EDGE : MCPWM_NEG_EDGE;
  cfg.cap_prescale = 1;
  cfg.capture_cb = mcpwmCaptureIsr;
  cfg.user_data = s;
  if (mcpwm_gpio_init(unit, static_cast<mcpwm_io_signals_t>(MCPWM_CAP_0 + ch), pin) != ESP_OK ||
      mcpwm_capture_enable_channel(unit, ch, &cfg) != ESP_OK)
  {
    s->pin = -1;
    return false;
  }
  return true;
}

void Gpio::detachCaptureInterrupt(uint8_t pin)
{
  auto* s = Detail::captureFind(pin);
  if (!s) return;
  const size_t idx = s - Detail::captureSlots;
  mcpwm_capture_disable_channel(static_cast<mcpwm_unit_t>(idx / SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER),
                                static_cast<mcpwm_capture_channel_id_t>(idx % SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER));
  s->pin = -1;
}

#else

bool Gpio::attachCaptureInterrupt(uint8_t pin, CaptureHandler handler, void* arg, InterruptMode mode)
{
  return Detail::captureAttachFallback(pin, handler, arg, mode);
}

void Gpio::detachCaptureInterrupt(uint8_t pin)
{
  Detail::captureDetachFallback(pin);
}

#endif

} // namespace Espfc::Hal

#endif
