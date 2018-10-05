#ifndef _ESP_TIMER8266_H_
#define _ESP_TIMER8266_H_

#if defined(ESP8266)

#include <cstdint>
#include <user_interface.h>
#include <Arduino.h>
#include "EspTimer.h"

#define ETS_FRC_TIMER2_INUM 10

#define TIMER0_WAIT_SHORT_COMP 10UL
#define TIMER0_WAIT_EDGE 240UL
#define TIMER0_WAIT_COMP 200UL
#define TIMER1_WAIT_EDGE 140UL
#define TIMER1_WAIT_COMP 115UL

enum TimerId {
  ESP_TIMER0 = 0,
  ESP_TIMER1,
  ESP_TIMER2,
};

class EspTimer8266: public EspTimer
{
  public:
    EspTimer8266(int timer): _timer(timer) {}

    void begin(int timer, callback_ptr cb, void * arg);
    void end();

    bool write(uint32_t ticks) ICACHE_RAM_ATTR;
    uint32_t usToTicks(uint32_t us) const ICACHE_RAM_ATTR;
    int32_t minTicks() const ICACHE_RAM_ATTR;

    int timer() const ICACHE_RAM_ATTR
    {
      return _timer;
    }
    
  private:  
    int _timer;
};

#endif

#endif