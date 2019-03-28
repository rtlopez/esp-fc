#ifndef _ESP_TIMER32_H_
#define _ESP_TIMER32_H_

#if defined(ESP32)

#include <cstdint>
#include <esp32-hal-timer.h>

#include "EspTimer.h"

enum EspTimerId {
  ESP_TIMER0 = 0,
  ESP_TIMER1,
  ESP_TIMER2,
  ESP_TIMER3,
  ESP_TIMER_COUNT
};

/*
typedef void (*__fn)(void);
static EspTimer * __timers[ESP_TIMER_COUNT] = { 0, 0, 0, 0 };
static void __isr0() { if(!__timers[0]) return; __timers[0]->execute(); }
static void __isr1() { if(__timers[1]) __timers[1]->execute(); }
static void __isr2() { if(__timers[2]) __timers[2]->execute(); }
static void __isr3() { if(__timers[3]) __timers[3]->execute(); }
static __fn __isrs[ESP_TIMER_COUNT] = { __isr0, __isr1, __isr2, __isr3 };
*/

class EspTimer32: public EspTimer
{
  public:
    EspTimer32(): _timer(-1), _dev(nullptr) {}

    void begin(EspTimerId timer, callback_ptr0 cb)
    {
      end();
      //_fn = cb;
      //_arg = arg;
      _timer = timer;
      //__timers[_timer] = this;
      _dev = timerBegin(timer, 1, true);
      timerAttachInterrupt(_dev, cb, true);
      timerAlarmWrite(_dev, 10000, false);
      timerAlarmEnable(_dev);
    }

    void end()
    {
      if(_timer == -1) return;
      timerEnd(_dev);
      _dev = nullptr;
      //__timers[_timer] = nullptr;
      _timer = -1;
    }

    bool write(uint64_t ticks)
    {
      if(_timer == -1) return false;
      timerAlarmWrite(_dev, ticks, true);
      return true;
    }

  private:
    int _timer;
    hw_timer_t * _dev;
};

#endif

#endif