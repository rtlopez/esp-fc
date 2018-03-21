#ifndef _ESC_DRIVER_ESP8266_H_
#define _ESC_DRIVER_ESP8266_H_

#if defined(ESP8266)

#include "EscDriver.h"
#include <Arduino.h>

//#define USE_FRC1
//#define USE_FRC1_NMI
#define USE_FRC2
//#define USE_FRC0 // cycle counter
//#define USE_UNIT // unit test

#if defined(USE_FRC0)
  #define TIMER_CLOCK_FREQ F_CPU
  #define TIMER_TICKS_MIN 250L
#elif defined(USE_UNIT)
  #define TIMER_CLOCK_FREQ 80000000L
  #define TIMER_TICKS_MIN 150L
#else
  #define TIMER_CLOCK_FREQ APB_CLK_FREQ
  #define TIMER_TICKS_MIN 150L
#endif

#define ETS_FRC_TIMER2_INUM 10
#define usToTicksReal(us) (TIMER_CLOCK_FREQ / 1000000L * us) // timer1, timer2

#define DELTA_TICKS_MAX usToTicksReal(50000)
#define DELTA_TICKS_MIN 5

class EscDriverEsp8266
{
  public:
    class Slot
    {
      public:
        Slot(): pin(-1), pulse(0) {}
        int pin;
        int pulse;
        bool operator<(const Slot& rhs) const
        {
          if(!active()) return false;
          else if(!rhs.active()) return true;
          return this->pulse < rhs.pulse;
        }
        inline bool active() const { return pin != -1; }
    };

    class Item
    {
      public:
        Item(): set_mask(0), clr_mask(0), ticks(0), last(0) {}
        uint32_t set_mask; // pin set mask
        uint32_t clr_mask; // pin clear mask
        int32_t ticks;    // delay to next cycle
        uint32_t last;
        void reset() { *this = Item(); }
    };

    typedef uint32_t mask_t;

    EscDriverEsp8266();

    int begin(EscProtocol protocol, bool async, int16_t rate);
    void end();

    int attach(size_t channel, int pin, int pulse) ICACHE_RAM_ATTR;

    int write(size_t channel, int pulse) ICACHE_RAM_ATTR;
    void apply() ICACHE_RAM_ATTR;

    static void handle(void * p) ICACHE_RAM_ATTR;

  private:

    void commit() ICACHE_RAM_ATTR;
    uint32_t usToTicks(uint32_t us) ICACHE_RAM_ATTR;

    uint16_t dshotEncode(const uint16_t value);
    void dshotWrite();

    volatile bool _busy;
    bool _async;
    EscProtocol _protocol;
    int _rate;
    int32_t _interval;
    int32_t _intervalMin; // for brushed
    int32_t _intervalMax; // for brushed

    int _dh;
    int _dl;

    Slot _slots[ESC_CHANNEL_COUNT];
    Item _items[ESC_CHANNEL_COUNT * 2];

    static const size_t DSHOT_BIT_COUNT = 16;
    mask_t dshotSetMask[DSHOT_BIT_COUNT];
    mask_t dshotClrMask[DSHOT_BIT_COUNT * 2];

    static EscDriverEsp8266 * _instance;
};

#endif // ESP8266

#endif
