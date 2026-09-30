#include "Device/Input/InputPPM.hpp"
#include "Hal/FastCode.hpp"
#include "Hal/Gpio.hpp"
#include "Hal/Time.hpp"

namespace Espfc::Device::Input {

void InputPPM::begin(int8_t pin, int mode)
{
  if (_pin != -1)
  {
    Hal::Gpio::detachInterrupt(_pin);
    _pin = -1;
  }
  if (pin != -1)
  {
    _pin = pin;
    _channel = 0;
    _last_tick = micros();
    for (size_t i = 0; i < CHANNELS; i++)
    {
      _channels[i].store(i == 2 ? 1000 : 1500, std::memory_order_relaxed); // throttle
    }
    Hal::Gpio::pinMode(_pin, Hal::Gpio::Input);
    Hal::Gpio::attachInterrupt(_pin, InputPPM::handle_isr, this, static_cast<Hal::Gpio::InterruptMode>(mode));
  }
}

InputStatus FAST_CODE_ATTR InputPPM::update()
{
  if (_new_data.load(std::memory_order_acquire))
  {
    _new_data.store(false, std::memory_order_relaxed);
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputPPM::get(uint8_t i) const
{
  return _channels[i].load(std::memory_order_relaxed);
}

void FAST_CODE_ATTR InputPPM::get(uint16_t* data, size_t len) const
{
  const auto* src = _channels;
  while (len--)
  {
    *data++ = (src++)->load(std::memory_order_relaxed);
  }
}

size_t InputPPM::getChannelCount() const
{
  return CHANNELS;
}

bool InputPPM::needAverage() const
{
  return true;
}

void ISR_CODE_ATTR InputPPM::handle()
{
  uint32_t now = micros();
  uint32_t width = now - _last_tick;

  _last_tick = now;

  if (width > 3000) // sync
  {
    _channel = 0;
    return;
  }

  if (_channel < CHANNELS) // ignore exceding channels
  {
    _channels[_channel].store(static_cast<int>(width), std::memory_order_relaxed);
  }
  if (_channel == 3)
  {
    _new_data.store(true, std::memory_order_release); // increase responsivnes for sticks channels
  }
  _channel++;
}

void ISR_CODE_ATTR InputPPM::handle_isr(void* args)
{
  if (args) static_cast<InputPPM*>(args)->handle();
}

} // namespace Espfc::Device::Input
