#pragma once

#if defined(ESP8266)

#include "EscDriver.h"

enum EscDriverTimer
{
  ESC_DRIVER_TIMER0,
  ESC_DRIVER_TIMER1,
  ESC_DRIVER_TIMER2
};

#define ETS_FRC_TIMER2_INUM 10

#define DELTA_TICKS_MAX ((APB_CLK_FREQ / 1000000L) * 50000L)
#define DELTA_TICKS_MIN 5

class EscDriverEsp8266: public EscDriverBase
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

    using mask_t = uint32_t;

    EscDriverEsp8266();

    int begin(const EscConfig& conf);
    void end();
    int attach(size_t channel, int pin, int pulse);
    int write(size_t channel, int pulse);
    void apply();
    int pin(size_t channel) const;
    uint32_t telemetry(size_t channel) const;
    static void handle(void * p, void * x);

  private:
    void commit();
    uint32_t usToTicks(uint32_t us);
    uint32_t usToTicksReal(EscDriverTimer timer, uint32_t us);
    int32_t minTicks(EscDriverTimer timer);
    void dshotWrite();

    static void _isr_init(EscDriverTimer timer, void * driver);
    static void _isr_begin(EscDriverTimer timer);
    static bool _isr_wait(EscDriverTimer timer, const uint32_t ticks);
    static void _isr_start(EscDriverTimer timer);
    static void _isr_reboot(void* p);
    static void _isr_end(EscDriverTimer timer, void* p);

    volatile bool _busy;
    bool _async;
    EscProtocol _protocol;
    int _rate;
    EscDriverTimer _timer;
    int32_t _interval;
    int32_t _intervalMin; // for brushed
    int32_t _intervalMax; // for brushed

    int _dh;
    int _dm;
    int _dl;

    Slot _slots[ESC_CHANNEL_COUNT];
    Item _items[ESC_CHANNEL_COUNT * 2];

    volatile Item * _it;
    const Item * _end;
};

#endif // ESP8266
