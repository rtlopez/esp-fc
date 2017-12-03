#ifndef _ESP8266_ESC_DRIVER_H_
#define _ESP8266_ESC_DRIVER_H_

#if defined(ESP8266)

#include "EscDriver.h"
#include "Arduino.h"

class Esp8266EscDriver
{
  public:
    class Slot
    {
      public:
        Slot(): pulse(1000), diff(0), pin(-1) {}
        volatile uint32_t pulse;
        volatile uint32_t diff;
        volatile uint8_t pin;
        bool operator<(const Slot& rhs) const { return this->pulse < rhs.pulse; }
    };

    Esp8266EscDriver();

    int begin(EscProtocol protocol, bool async, int16_t rate);

    int attach(size_t channel, int pin, int pulse);

    int write(size_t channel, int pulse) ICACHE_RAM_ATTR;

    void apply() ICACHE_RAM_ATTR;

    void commit() ICACHE_RAM_ATTR;

    void handle() ICACHE_RAM_ATTR;

    static void handle_isr() ICACHE_RAM_ATTR;

    void trigger() ICACHE_RAM_ATTR;

    Slot * begin() ICACHE_RAM_ATTR
    {
      return _slots;
    }

    Slot * end() ICACHE_RAM_ATTR
    {
      return _slots + ESC_CHANNEL_COUNT;
    }

    uint32_t space() const ICACHE_RAM_ATTR
    {
      return _space;
    }

    bool async() const ICACHE_RAM_ATTR
    {
      return _async;
    }

    inline uint32_t usToTicks(uint32_t us, bool real = false) const ICACHE_RAM_ATTR
    {
      //uint32_t ticks = microsecondsToClockCycles(us); // timer0
      uint32_t ticks = APB_CLK_FREQ / 1000000L * us; // timer1
      if(!real)
      {
        switch(_protocol)
        {
          case ESC_PROTOCOL_ONESHOT125:
            ticks = ticks >> 3;
            break;
          default:
            break;
        }
      }
      return ticks - 180; // ~180 cycles compensation for isr trigger
    }

  private:
    Slot _buffer[ESC_CHANNEL_COUNT];
    Slot _slots[ESC_CHANNEL_COUNT];

    EscProtocol _protocol;
    bool _async;
    int16_t _rate;
    uint32_t _interval;
    uint32_t _space;
    volatile bool _isr_busy;

    static Esp8266EscDriver * _instance;
};

#endif // ESP8266

#endif
