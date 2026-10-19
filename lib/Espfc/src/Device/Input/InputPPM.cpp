#include "Device/Input/InputPPM.hpp"
#include "Hal/FastCode.hpp"
#include "Hal/Gpio.hpp"

namespace Espfc::Device::Input {

void InputPPM::begin(int8_t pin, PPMInvert invert)
{
  if (_pin != -1)
  {
    Hal::Gpio::detachCaptureInterrupt(_pin);
    _pin = -1;
  }
  _read_count = _write_count.load(std::memory_order_relaxed);
  if (pin != -1)
  {
    _pin = pin;
    _channel = 0;
    for (size_t i = 0; i < CHANNELS; i++)
    {
      _channels[i].store(i == 2 ? 1000 : 1500, std::memory_order_relaxed); // throttle
    }
    Hal::Gpio::pinMode(_pin, Hal::Gpio::Input);
    if (!Hal::Gpio::attachCaptureInterrupt(_pin, InputPPM::handle_isr, this,
                                           invert == PPM_MODE_INVERTED ? Hal::Gpio::Falling : Hal::Gpio::Rising))
    {
      _pin = -1;
    }
  }
}

InputStatus FAST_CODE_ATTR InputPPM::update()
{
  auto writeCount = _write_count.load(std::memory_order_acquire);
  if (writeCount == _read_count) return INPUT_IDLE;
  _read_count = writeCount;
  return INPUT_RECEIVED;
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

void ISR_CODE_ATTR InputPPM::handle(uint32_t width)
{
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
    // report after throttle channel to increase responsiveness
    // we cannot use fetch_add() as not all architectures support it fully
    _write_count.store(_write_count.load(std::memory_order_relaxed) + 1, std::memory_order_release);
  }

  _channel++;
}

void ISR_CODE_ATTR InputPPM::handle_isr(void* args, uint32_t width)
{
  if (args) static_cast<InputPPM*>(args)->handle(width);
}

} // namespace Espfc::Device::Input
