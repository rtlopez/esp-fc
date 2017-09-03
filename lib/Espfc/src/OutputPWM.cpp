#include "OutputPWM.h"

#include <algorithm>

namespace Espfc {

OutputPWM PWM;

static const int frameComp = 1;
static int frameCount = 0;

static void _pwm_timer0_ISR(void) ICACHE_RAM_ATTR;
void _pwm_timer0_isr(void)
{
  OutputPWM::Slot * slot = PWM.getCurrentSlot();
  if(slot == NULL)  // space pulse to align frequency
  {
    PWM.nextSlot();
    timer0_write(ESP.getCycleCount() + PWM.usToTicks(std::max(PWM._frameLength - frameCount, 2 * frameComp)));
    frameCount = 0;
    return;
  }
  if(slot->val == LOW) // enable pin
  {
    int pulse = slot->pulse - frameComp;
    slot->val = HIGH;
    digitalWrite(slot->pin, slot->val);
    frameCount += pulse + 2 * frameComp;
    timer0_write(ESP.getCycleCount() + PWM.usToTicks(pulse));
  }
  else                 // disable pin
  {
    slot->val = LOW;
    digitalWrite(slot->pin, slot->val);
    PWM.nextSlot();
    timer0_write(ESP.getCycleCount() + PWM.usToTicks(frameComp));
  }
}

OutputPWM::OutputPWM()
{
  for(size_t i = 0; i < OUTPUT_CHANNELS; ++i)
  {
    _slots[i] = Slot();
  }
  _currentSlot = 0;
  _frameLength = 1000000 / 50;
}

int OutputPWM::begin(int rate)
{
  if(rate >= 50 && rate < 490)
  {
    _frameLength = 1000000 / rate;
  }
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(_pwm_timer0_isr);
  interrupts();
  timer0_write(ESP.getCycleCount() + usToTicks(20000));
  return 1;
}

}
