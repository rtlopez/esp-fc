#include "OutputPWMFast.h"

namespace Espfc {

OutputPWMFast PWMfast;

static volatile bool _isr_busy = false;
static void _pwm_fast_handle_isr(void) ICACHE_RAM_ATTR;

static inline uint32_t usToTicks(uint32_t us)
{
  //return microsecondsToClockCycles(us); // timer0
  return APB_CLK_FREQ / 1000000L * us; // timer1
  //return F_CPU / 1000000L / 2 * us; // timer1
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
  //timer0_write(ESP.getCycleCount() + usToTicks(us));
  //timer1_write(usToTicks(us));
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

void _pwm_fast_handle_isr(void)
{
  static OutputPWMFast::Slot * end = PWMfast.end();
  static OutputPWMFast::Slot * it = NULL;

  // start cycle
  if(!it)
  {
    _isr_busy = true;
    for(it = PWMfast.begin(); it != end; ++it)
    {
      if(it->pin == -1) continue;
      digitalWriteFast(it->pin, HIGH);
    }
    it = PWMfast.begin();
    while(it->pin == -1 && it != end) ++it;
    if(it != end) timer_write(it->diff);
    return;
  }

  // suppress similar pulses
  while(it != end)
  {
    digitalWriteFast(it->pin, LOW);
    ++it;
    if(it == end) break;
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
}

void OutputPWMFast::trigger()
{
  //PIN_DEBUG(true);
  //PIN_DEBUG(false);
  if(_isr_busy == true) return;
  _pwm_fast_handle_isr();
}

OutputPWMFast::OutputPWMFast(): _rate(50), _protocol(OUTPUT_PWM)
{
  for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
  {
    _slots[i] = Slot();
    _buffer[i] = Slot();
  }
}

int OutputPWMFast::begin(int rate, OutputProtocol protocol)
{
  _protocol = protocol;
  _rate = rate;
  timer_init();
  return 1;
}

}
