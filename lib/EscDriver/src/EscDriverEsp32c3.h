#pragma once

#if defined(ESP32C3)

#include "EscDriver.h"

enum EscDriverTimer
{
  ESC_DRIVER_TIMER0,
  ESC_DRIVER_TIMER1,
};

class EscDriverEsp32c3: public EscDriverBase
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

    EscDriverEsp32c3();

    int begin(const EscConfig& conf);
    void end();
    int attach(size_t channel, int pin, int pulse) IRAM_ATTR;
    int write(size_t channel, int pulse) IRAM_ATTR;
    void apply() IRAM_ATTR;
    int pin(size_t channel) const;
    uint32_t telemetry(size_t channel) const;
    static bool handle(void * p) IRAM_ATTR;

  private:
    void commit() IRAM_ATTR;
    uint32_t usToTicks(uint32_t us) IRAM_ATTR;
    uint32_t usToTicksReal(EscDriverTimer timer, uint32_t us) IRAM_ATTR;
    int32_t minTicks(EscDriverTimer timer) IRAM_ATTR;
    void dshotWrite() IRAM_ATTR;

    static void _isr_init(EscDriverTimer timer, void * driver);
    static bool _isr_wait(EscDriverTimer timer, const uint32_t ticks) IRAM_ATTR;
    static void _isr_start(EscDriverTimer timer) IRAM_ATTR;
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
    int _dl;

    Slot _slots[ESC_CHANNEL_COUNT];
    Item _items[ESC_CHANNEL_COUNT * 2];

    volatile Item * _it;
    const Item * _end;

    mask_t dshotSetMask[DSHOT_BIT_COUNT];
    mask_t dshotClrMask[DSHOT_BIT_COUNT * 2];

    //static EscDriverEsp32c3 * _instance;
};

#endif // ESP32C3
