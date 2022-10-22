#if defined(ARCH_RP2040)

#include "EscDriverRP2040.h"
#include <hardware/gpio.h>
#include <hardware/pwm.h>

int EscDriverRP2040::attach(size_t channel, int pin, int pulse)
{
  if(channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pin = pin;
  _slots[channel].pulse = usToTicks(pulse);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  if(_protocol < ESC_PROTOCOL_DSHOT150)
  {
    _slots[channel].slice = pwm_gpio_to_slice_num(pin);
    _slots[channel].channel = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, _divider);    /// Setting the divider to slow down the clock
    pwm_config_set_wrap(&config, _interval);         /// setting the Wrap time

    gpio_set_function(pin, GPIO_FUNC_PWM);           /// Set the pin to be PWM
    pwm_init(_slots[channel].slice, &config, true);  /// start PWM
  }
  return 1;
}

int EscDriverRP2040::write(size_t channel, int pulse)
{
  if(channel >= ESC_CHANNEL_COUNT) return 0;
  _slots[channel].pulse = usToTicks(pulse);
  return 1;
}

void EscDriverRP2040::apply()
{
  if(_protocol >= ESC_PROTOCOL_DSHOT150 && _protocol <= ESC_PROTOCOL_DSHOT600)
  {
    dshotWrite();
    return;
  }
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;
    pwm_set_chan_level(_slots[i].slice, _slots[i].channel, _slots[i].pulse);
  }
}

uint32_t EscDriverRP2040::usToTicks(uint32_t us)
{
  uint32_t ticks = 0;
  switch(_protocol)
  {
    case ESC_PROTOCOL_ONESHOT125:
      ticks = map(us, 1000, 2000, usToTicksReal(125), usToTicksReal(250));
      break;
    case ESC_PROTOCOL_ONESHOT42:
      ticks = map(us, 1000, 2000, usToTicksReal(42), usToTicksReal(84));
      break;
    case ESC_PROTOCOL_MULTISHOT:
      ticks = map(us, 1000, 2000, usToTicksReal(5), usToTicksReal(25));
      break;
    case ESC_PROTOCOL_BRUSHED:
      ticks = map(constrain(us, 1000, 2000), 1000, 2000, 0, _interval);
      break;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    //case ESC_PROTOCOL_DSHOT1200:
      ticks = us;
      break;
    default:
      ticks = usToTicksReal(us); // PWM
      break;
  }
  return ticks;
}

uint32_t EscDriverRP2040::usToTicksReal(uint32_t us)
{
  uint64_t t = (uint64_t)us * (F_CPU / _divider);
  return t / 1000000ul;
}

EscDriverRP2040::EscDriverRP2040(): _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50), _timer(ESC_DRIVER_TIMER1), _divider(F_CPU / 3 / 1000000ul), _interval(usToTicksReal(1000000uL / _rate))
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i) _slots[i] = Slot();
}

int EscDriverRP2040::begin(EscProtocol protocol, bool async, int16_t rate, EscDriverTimer timer)
{
  _timer = timer;
  _protocol = ESC_PROTOCOL_SANITIZE(protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : async; // force async for brushed
  _rate = constrain(rate, 50, 8000);
  _interval = usToTicksReal(1000000ul / _rate);

  switch(_protocol)
  {
    // pulse delays experimentally selected
    case ESC_PROTOCOL_DSHOT150:
      {
        const float base = 140;
        _dh = lrintf(base * 0.35f);
        _dl = lrintf(base * 0.30f);
      }
      break;
    case ESC_PROTOCOL_DSHOT300:
      {
        const float base = 68;
        _dh = lrintf(base * 0.35f);
        _dl = lrintf(base * 0.30f);
      }
      break;
    case ESC_PROTOCOL_DSHOT600:
    //case ESC_PROTOCOL_DSHOT1200:
    case ESC_PROTOCOL_PROSHOT:
      {
        const float base = 31;
        _dh = lrintf(base * 0.36f);
        _dl = lrintf(base * 0.28f);
      }
      break;
    case ESC_PROTOCOL_DISABLED:
      break;
    default: // analog
      break;
  }
  return 1;
}

void EscDriverRP2040::end()
{
  for(size_t i = 0; i < ESC_CHANNEL_COUNT; ++i)
  {
    if(!_slots[i].active()) continue;
    digitalWrite(_slots[i].pin, LOW);
  }
}

static inline void dshotDelay(int delay)
{
  while(delay--) __asm__ __volatile__ ("nop");
}

void EscDriverRP2040::dshotWrite()
{
  // zero mask arrays
  mask_t * sm = dshotSetMask;
  mask_t * cm = dshotClrMask;
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    *sm = 0; sm++;
    *cm = 0; cm++;
    *cm = 0; cm++;
  }

  // compute bits
  for(size_t c = 0; c < ESC_CHANNEL_COUNT; c++)
  {
    if(_slots[c].pin > 28 || _slots[c].pin < 0) continue;
    mask_t mask = (1U << _slots[c].pin);
    int pulse = constrain(_slots[c].pulse, 0, 2000);
    int value = 0; // disarmed
    // scale to dshot commands (0 or 48-2047)
    if(pulse > 1000)
    {
      value =  PWM_TO_DSHOT(pulse);
    }
    uint16_t frame = dshotEncode(value);
    for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
    {
      int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
      dshotSetMask[i] |= mask;
      dshotClrMask[(i << 1) + val] |= mask;
    }
  }

  // write output
  sm = dshotSetMask;
  cm = dshotClrMask;
  for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    gpio_set_mask(*sm); sm++; dshotDelay(_dh);
    gpio_clr_mask(*cm); cm++; dshotDelay(_dh);
    gpio_clr_mask(*cm); cm++; dshotDelay(_dl);
  }
}

#endif // ARCH_RP2040
