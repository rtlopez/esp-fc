#ifndef _ESC_DRIVER_ESP8266_H_
#define _ESC_DRIVER_ESP8266_H_

#if defined(ESP8266)

#include "EscDriver.h"
#include "EspTimer8266.h"

enum EscDriverTimer
{
  ESC_DRIVER_TIMER0 = ESP_TIMER0,
  ESC_DRIVER_TIMER1,
  ESC_DRIVER_TIMER2
};

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

    typedef uint32_t mask_t;

    EscDriverEsp8266();

    int begin(EscProtocol protocol, bool async, int16_t rate, EscDriverTimer timer);
    void end();
    int attach(size_t channel, int pin, int pulse);

    int write(size_t channel, int pulse) ICACHE_RAM_ATTR;
    void apply() ICACHE_RAM_ATTR;
  
    void commit() ICACHE_RAM_ATTR;
    uint32_t usToTicks(uint32_t us) ICACHE_RAM_ATTR;
    void dshotWrite() ICACHE_RAM_ATTR;

    volatile Item * it() ICACHE_RAM_ATTR { return _it; }
    bool working() const ICACHE_RAM_ATTR { return _it && _it != _end; }
    void done() ICACHE_RAM_ATTR { _it = nullptr; }
    void rewind() ICACHE_RAM_ATTR { _it = _items; }
    void next() ICACHE_RAM_ATTR { _it++; }
    void busy(bool busy) ICACHE_RAM_ATTR { _busy = busy; }
    bool wait(uint32_t ticks) ICACHE_RAM_ATTR { return _timer.write(ticks); }

  private:
    volatile bool _busy;
    bool _async;
    EscProtocol _protocol;
    int _rate;
    EspTimer8266 _timer;

    int32_t _interval;
    int32_t _intervalMin; // for brushed
    int32_t _intervalMax; // for brushed

    Slot _slots[ESC_CHANNEL_COUNT];
    Item _items[ESC_CHANNEL_COUNT * 2];

    volatile Item * _it;
    const Item * _end;

    // dshot
    int _dh;
    int _dl;
    mask_t dshotSetMask[DSHOT_BIT_COUNT];
    mask_t dshotClrMask[DSHOT_BIT_COUNT * 2];
};

#endif // ESP8266

#endif
