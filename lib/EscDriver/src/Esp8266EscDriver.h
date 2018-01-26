#ifndef _ESP8266_ESC_DRIVER_H_
#define _ESP8266_ESC_DRIVER_H_

#if defined(ESP8266)

#include "EscDriver.h"
#include "Arduino.h"

//#define usToTicksReal(us) (microsecondsToClockCycles(us)) // timer0
//#define usToTicksReal(us) (F_CPU / 1000000L * us) // timer0
#define usToTicksReal(us) (APB_CLK_FREQ / 1000000L * us) // timer1

class Esp8266EscDriver
{
  public:
    class Slot
    {
      public:
        Slot(): pulse(0), diff(0), pin(-1) {}
        volatile uint32_t pulse;
        volatile uint32_t diff;
        volatile uint8_t pin;
        bool operator<(const Slot& rhs) const
        {
          if(!active()) return false;
          else if(!rhs.active()) return true;
          return this->pulse < rhs.pulse;
        }
        inline bool active() const { return pin != -1; }
    };

    Esp8266EscDriver();

    int begin(EscProtocol protocol, bool async, int16_t rate);

    int attach(size_t channel, int pin, int pulse);

    int write(size_t channel, int pulse);

    void apply() ICACHE_RAM_ATTR;

    void commit() ICACHE_RAM_ATTR;

    void handle() ICACHE_RAM_ATTR;

    void handle_serial() ICACHE_RAM_ATTR;

    static void handle_isr() ICACHE_RAM_ATTR;

    const Slot * begin() const ICACHE_RAM_ATTR
    {
      return _slots;
    }

    const Slot * end() const ICACHE_RAM_ATTR
    {
      return _slots + ESC_CHANNEL_COUNT;
    }

    inline uint32_t usToTicks(uint32_t us) const
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
          ticks = map(us, 1000, 2000, usToTicksReal(5), usToTicksReal(20));
          break;
        case ESC_PROTOCOL_BRUSHED:
          ticks = map(constrain(us, 1001, 1999), 1000, 2000, 0, _interval); // strange behaviour at edges of range
          break;
        default:
          ticks = usToTicksReal(us); // PWM
          break;
      }
      return ticks;
    }

    static const uint32_t PULSE_COMPENSATION = 290; // ~200 cycles compensation for isr trigger

  private:
    Slot _buffer[ESC_CHANNEL_COUNT];
    Slot _slots[ESC_CHANNEL_COUNT];

    EscProtocol _protocol;
    bool _async;
    int16_t _rate;
    uint32_t _interval;
    uint32_t _interval_us;
    uint32_t _space;
    volatile bool _isr_busy;

    static Esp8266EscDriver * _instance;
};

#endif // ESP8266

#endif
