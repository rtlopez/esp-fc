#include "OutputPWMFast.h"

namespace Espfc {

OutputPWMFast PWMfast;

static volatile OutputPWMFast::Slot * it = NULL;
static volatile OutputPWMFast::Slot * end = NULL;
static volatile bool _isr_busy = false;

static void _pwm_fast_handle_isr(void) ICACHE_RAM_ATTR;
void _pwm_fast_handle_isr(void)
{
  if(!end) end = PWMfast.end();

  if(!it) // start cycle
  {
    _isr_busy = true;
    for(it = PWMfast.begin(); it != end; ++it)
    {
      if(it->pin == -1) continue;
      digitalWrite(it->pin, HIGH);
    }
    it = PWMfast.begin();
    while(it->pin == -1 && it != end) ++it;
    if(it != end) timer0_write(ESP.getCycleCount() + PWMfast.usToTicks(it->diff));
    return;
  }

  // suppress similar pulses
  while(it != end)
  {
    digitalWrite(it->pin, LOW);
    ++it;
    if(it->pin == -1) continue;
    if(it->diff > 2)
    {
      // jump to next cycle
      timer0_write(ESP.getCycleCount() + PWMfast.usToTicks(it->diff));
      return;
    }
  }

  // finish cycle
  _isr_busy = false;
  it = NULL;
}

void OutputPWMFast::trigger()
{
  if(_isr_busy) return;
  _pwm_fast_handle_isr();
}

OutputPWMFast::OutputPWMFast()
{
  for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
  {
    _slots[i] = Slot();
    _buffer[i] = Slot();
  }
}

int OutputPWMFast::begin(int rate)
{
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(_pwm_fast_handle_isr);
  interrupts();
  timer0_write(ESP.getCycleCount() + PWMfast.usToTicks(20000));
  return 1;
}

}
