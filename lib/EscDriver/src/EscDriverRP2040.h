#ifndef _ESC_DRIVER_ESP8266_H_
#define _ESC_DRIVER_ESP8266_H_

#if defined(ARCH_RP2040)

#include "EscDriver.h"

// TODO: https://cocode.se/linux/raspberry/pwm.html

enum EscDriverTimer
{
  ESC_DRIVER_TIMER0,
  ESC_DRIVER_TIMER1
};

#define DELTA_TICKS_MAX ((F_CPU / 1000000L) * 50000L)
#define DELTA_TICKS_MIN 5

class EscDriverRP2040: public EscDriverBase
{
  public:
    class Slot
    {
      public:
        Slot(): pin(-1), pulse(0), slice(0), channel(0) {}
        int pin;
        int pulse;
        int slice;
        int channel;
        inline bool active() const { return pin != -1; }
    };

    typedef uint32_t mask_t;

    EscDriverRP2040();

    int begin(EscProtocol protocol, bool async, int16_t rate, EscDriverTimer timer);
    void end();
    int attach(size_t channel, int pin, int pulse) IRAM_ATTR;
    int write(size_t channel, int pulse) IRAM_ATTR;
    void apply() IRAM_ATTR;
    static void handle(void * p, void * x) IRAM_ATTR;

  private:
    uint32_t usToTicks(uint32_t us) IRAM_ATTR;
    uint32_t usToTicksReal(uint32_t us) IRAM_ATTR;
    void dshotWrite() IRAM_ATTR;

    EscProtocol _protocol;
    bool _async;
    int _rate;
    EscDriverTimer _timer;
    uint16_t _divider;
    uint32_t _interval;

    Slot _slots[ESC_CHANNEL_COUNT];

    int _dh;
    int _dl;
    mask_t dshotSetMask[DSHOT_BIT_COUNT];
    mask_t dshotClrMask[DSHOT_BIT_COUNT * 2];
};

#endif // ARCH_RP2040

#endif
