#if defined(ESP8266)

#include "EspTimer8266.h"

static ICACHE_RAM_ATTR void __handleIsr(void * arg)
{
  EspTimer8266 * t = (EspTimer8266 *)arg;
  switch(t->timer())
  {
    case ESP_TIMER2:
      T2I = 0;
      break;
    case ESP_TIMER1:
      TEIE &= ~TEIE1; //14
      T1I = 0; //9
      break;
    default:
      break;
  }
  t->execute();
}

void EspTimer8266::begin(EspTimerId timer, callback_ptr cb, void * arg)
{
  _timer = timer;
  _fn = cb;
  _arg = arg;
  switch(_timer)
  {
    case ESP_TIMER2:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER2_INUM);
      T2C = 0;
      T2I = 0;
      ets_isr_attach(ETS_FRC_TIMER2_INUM, __handleIsr, this);
      ETS_INTR_ENABLE(ETS_FRC_TIMER2_INUM);
      T2C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
      T2I = 0;
      ETS_INTR_UNLOCK();
      break;
    case ESP_TIMER1:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
      T1C = 0;
      T1I = 0;
      ets_isr_attach(ETS_FRC_TIMER1_INUM, __handleIsr, this);
      ETS_INTR_ENABLE(ETS_FRC_TIMER1_INUM);
      T1C = (1 << TCTE) | (TIM_DIV1 << TCPD) | (TIM_EDGE << TCIT) | (TIM_SINGLE << TCAR);
      T1I = 0;
      ETS_INTR_UNLOCK();
      break;
    case ESP_TIMER0:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_CCOMPARE0_INUM);
      ets_isr_attach(ETS_CCOMPARE0_INUM, __handleIsr, this);
      ETS_INTR_ENABLE(ETS_CCOMPARE0_INUM);
      ETS_INTR_UNLOCK();
      break;
  }
}

void EspTimer8266::end()
{
  switch(_timer)
  {
    case ESP_TIMER2:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER2_INUM);
      //ets_isr_attach(ETS_FRC_TIMER2_INUM, _isr_reboot, p);
      //ETS_INTR_ENABLE(ETS_FRC_TIMER2_INUM);
      ETS_INTR_UNLOCK();
      break;
    case ESP_TIMER1:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_FRC_TIMER1_INUM);
      ETS_INTR_UNLOCK();
      break;
    case ESP_TIMER0:
      ETS_INTR_LOCK();
      ETS_INTR_DISABLE(ETS_CCOMPARE0_INUM);
      ETS_INTR_UNLOCK();
      break;
  }
}

bool EspTimer8266::write(uint32_t ticks)
{
  switch(_timer)
  {
    case ESP_TIMER2:
      //if(ticks > 128) { // yield
      if(ticks > TIMER1_WAIT_EDGE) { // yield
        T2A = ((uint32_t)T2V + ticks - TIMER1_WAIT_COMP);
        //T2A = ((uint32_t)T2V + ticks) - 120UL;
        //T2L = 0;
        //T2A = (ticks - 100UL);
        T2I = 0;
        return true;
      }
      break;
    case ESP_TIMER1:
      if(ticks > TIMER1_WAIT_EDGE) { // yield
        T1L = ((ticks - TIMER1_WAIT_COMP) & 0x7FFFFF); //23
        TEIE |= TEIE1; //13
        return true;
      }
      break;
    case ESP_TIMER0:
      if(ticks > TIMER0_WAIT_EDGE) { // yield
        timer0_write((ESP.getCycleCount() + ticks - TIMER0_WAIT_COMP));
        return true;
      }
      break;
  }

  if(ticks > 20) { // or delay
    const uint32_t end = ESP.getCycleCount() + ticks - TIMER0_WAIT_SHORT_COMP;
    while(ESP.getCycleCount() < end) {
      __asm__ __volatile__ ("nop");
    };
  }

  return false;
}

uint32_t EspTimer8266::usToTicks(uint32_t us) const
{
  switch(_timer)
  {
    case ESP_TIMER0:
      return (F_CPU / 1000000L) * us;
    default:
      return (APB_CLK_FREQ / 1000000L) * us;
  }
}

int32_t EspTimer8266::minTicks() const
{
  switch(_timer)
  {
    case ESP_TIMER0:
      return 250L;
    default:
      return 150L;
  }
}

#endif