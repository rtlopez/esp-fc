#include "OutputPWMFast.h"

namespace Espfc {

OutputPWMFast PWMfast;

static void _pwm_fast_handle_isr(void) ICACHE_RAM_ATTR;

void _pwm_fast_handle_isr(void)
{
  PWMfast.handle();
}

static void timer_init()
{
  noInterrupts();
  //timer0_isr_init();
  //timer0_attachInterrupt(_pwm_fast_handle_isr);
  timer1_isr_init();
  timer1_attachInterrupt(_pwm_fast_handle_isr);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
  interrupts();
}

static inline void timer_write(uint32_t ticks)
{
  //timer0_write(ESP.getCycleCount() + ticks);
  timer1_write(ticks);
}

static inline void delay_ticks(uint32_t ticks)
{
  if(ticks < 20) return;
  uint32_t end = ESP.getCycleCount() + ticks - 10;
  while(ESP.getCycleCount() < end)
  {
    __asm__ volatile ("nop");
  };
}

static inline void digitalWriteFast(uint8_t pin, uint8_t val)
{
  //digitalWrite(pin, val);
  if(pin < 16)
  {
    if(val) GPOS = (1 << pin);
    else GPOC = (1 << pin);
  }
  else if(pin == 16)
  {
    if(val) GP16O |= 1;
    else GP16O &= ~1;
  }
}

void OutputPWMFast::handle(void)
{
  static const OutputPWMFast::Slot * en = end();
  static const OutputPWMFast::Slot * it = NULL;

  // start cycle
  if(!it)
  {
    _isr_busy = true;
    if(_async)
    {
      commit();
    }
    for(it = begin(); it != en; ++it)
    {
      if(it->pin == -1) continue;
      digitalWriteFast(it->pin, HIGH);
    }
    it = begin();
    while(it->pin == -1 && it != en) ++it;
    if(it != en) timer_write(it->diff);
    return;
  }

  // suppress similar pulses
  while(it != en)
  {
    digitalWriteFast(it->pin, LOW);
    ++it;
    if(it == en) break;
    if(it->pin == -1) continue;
    if(it->diff > 200)
    {
      // jump to next cycle
      timer_write(it->diff);
      return;
    }
    else
    {
      delay_ticks(it->diff);
    }
  }

  // finish cycle
  _isr_busy = false;
  it = NULL;

  // trigger next cycle
  if(_async)
  {
    timer_write(_space);
  }
}

void OutputPWMFast::trigger()
{
  //PIN_DEBUG(true);
  //PIN_DEBUG(false);
  if(_isr_busy == true) return;
  //_pwm_fast_handle_isr();
  handle();
}

void OutputPWMFast::commit()
{
  Slot tmp[OUTPUT_CHANNELS];
  std::copy(_buffer, _buffer + OUTPUT_CHANNELS, tmp);
  std::sort(tmp, tmp + OUTPUT_CHANNELS);
  Slot * end = tmp + OUTPUT_CHANNELS;
  Slot * prev = NULL;
  for(Slot * it = tmp; it != end; ++it)
  {
    if(it->pin == -1) continue;
    if(!prev) it->diff = it->pulse;
    else it->diff = it->pulse - prev->pulse;
    prev = it;
  }
  _space = prev ? _interval - prev->pulse : _interval; // after loop prev is the last item with longest pulse
  std::copy(tmp, tmp + OUTPUT_CHANNELS, _slots);
}

OutputPWMFast::OutputPWMFast(): _protocol(OUTPUT_PWM), _async(true), _rate(50), _isr_busy(false)
{
  _interval = usToTicks(1000000L / _rate, true);
  for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
  {
    _slots[i] = Slot();
    _buffer[i] = Slot();
  }
}

int OutputPWMFast::begin(OutputProtocol protocol, bool async, int16_t rate)
{
  _protocol = protocol;
  _async = async;
  _rate = rate;
  _interval = usToTicks(1000000L / _rate, true);
  timer_init();
  if(_async) timer_write(_interval);
  return 1;
}

}
