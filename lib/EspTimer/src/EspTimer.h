#ifndef _ESP_TIMER_H_
#define _ESP_TIMER_H_

class EspTimer
{
  public:
    typedef void (*callback_ptr)(void*);
    typedef void (*callback_ptr0)(void);

    void execute() ICACHE_RAM_ATTR
    {
      if(_fn) _fn(_arg);
    }

  protected:
    callback_ptr _fn;
    void * _arg;
};

#if defined(ESP32)

  #include "EspTimer32.h"
  #define EspTimerImpl EspTimer32

#elif defined(ESP8266)

  #include "EspTimer8266.h"
  #define EspTimerImpl EspTimer8266

#else

  #error "Unsupported platform"

#endif

#endif