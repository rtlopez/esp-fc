#include "Device/InputPWM.h"

#include <Arduino.h>

#include "Utils/MemoryHelper.h"

namespace Espfc {
namespace Device {

InputPWM::InputPWM()
{
  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i].pin = -1;
    _channels[i].riseUs = 0;
    _channels[i].lastUs = 0;
    _channels[i].widthUs = defaultValue(i);
    _channels[i].updated = false;

    _args[i].self = this;
    _args[i].index = static_cast<uint8_t>(i);
  }
}

uint16_t InputPWM::defaultValue(size_t channel)
{
  // esp-fc default raw receiver order uses channel 2 as throttle.
  return channel == 2 ? THROTTLE_LOW_US : MID_US;
}

void InputPWM::begin(const int8_t* pins, size_t len)
{
  for(size_t i = 0; i < CHANNELS; i++)
  {
    if(_channels[i].pin >= 0)
    {
      detachInterrupt(static_cast<uint8_t>(_channels[i].pin));
    }

    _channels[i].pin = -1;
    _channels[i].riseUs = 0;
    _channels[i].lastUs = 0;
    _channels[i].widthUs = defaultValue(i);
    _channels[i].updated = false;
  }

  if(!pins) return;

  const size_t count = len < CHANNELS ? len : CHANNELS;

  for(size_t i = 0; i < count; i++)
  {
    if(pins[i] < 0) continue;

    _channels[i].pin = pins[i];

#if defined(INPUT_PULLDOWN)
    pinMode(static_cast<uint8_t>(pins[i]), INPUT_PULLDOWN);
#else
    pinMode(static_cast<uint8_t>(pins[i]), INPUT);
#endif

#if defined(UNIT_TEST)
    // No hardware interrupt in unit tests.
#elif defined(ARCH_RP2040)
    attachInterruptParam(
      static_cast<uint8_t>(pins[i]),
      InputPWM::handleIsr,
      static_cast<PinStatus>(CHANGE),
      &_args[i]
    );
#else
    attachInterruptArg(
      static_cast<uint8_t>(pins[i]),
      InputPWM::handleIsr,
      &_args[i],
      CHANGE
    );
#endif
  }
}

InputStatus FAST_CODE_ATTR InputPWM::update()
{
  const uint32_t now = micros();

  bool hasConfiguredChannel = false;
  bool hasUpdate = false;

  for(size_t i = 0; i < CHANNELS; i++)
  {
    if(_channels[i].pin < 0) continue;

    hasConfiguredChannel = true;

    const uint32_t lastUs = _channels[i].lastUs;

    if(lastUs == 0)
    {
      return INPUT_LOST;
    }

    if(now - lastUs > SIGNAL_LOST_US)
    {
      return INPUT_LOST;
    }

    if(_channels[i].updated)
    {
      _channels[i].updated = false;
      hasUpdate = true;
    }
  }

  if(!hasConfiguredChannel)
  {
    return INPUT_LOST;
  }

  return hasUpdate ? INPUT_RECEIVED : INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputPWM::get(uint8_t channel) const
{
  if(channel >= CHANNELS)
  {
    return MID_US;
  }

  if(_channels[channel].pin < 0)
  {
    return defaultValue(channel);
  }

  return _channels[channel].widthUs;
}

void FAST_CODE_ATTR InputPWM::get(uint16_t* data, size_t len) const
{
  if(!data) return;

  for(size_t i = 0; i < len; i++)
  {
    data[i] = get(static_cast<uint8_t>(i));
  }
}

size_t InputPWM::getChannelCount() const
{
  return CHANNELS;
}

bool InputPWM::needAverage() const
{
  return true;
}

void IRAM_ATTR InputPWM::handle(uint8_t channel)
{
  if(channel >= CHANNELS) return;

  Channel& ch = _channels[channel];

  if(ch.pin < 0) return;

  const uint32_t now = micros();

  if(digitalRead(static_cast<uint8_t>(ch.pin)))
  {
    ch.riseUs = now;
    return;
  }

  const uint32_t riseUs = ch.riseUs;

  if(riseUs == 0) return;

  const uint32_t widthUs = now - riseUs;

  if(widthUs >= PULSE_MIN_US && widthUs <= PULSE_MAX_US)
  {
    ch.widthUs = static_cast<uint16_t>(widthUs);
    ch.lastUs = now;
    ch.updated = true;
  }
}

void IRAM_ATTR InputPWM::handleIsr(void* arg)
{
  const IsrArg* isrArg = reinterpret_cast<const IsrArg*>(arg);

  if(!isrArg || !isrArg->self) return;

  isrArg->self->handle(isrArg->index);
}

} // namespace Device
} // namespace Espfc