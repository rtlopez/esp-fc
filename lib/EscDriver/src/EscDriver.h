#ifndef _ESC_DRIVER_H_
#define _ESC_DRIVER_H_

#include "Arduino.h"

enum EscProtocol {
  ESC_PROTOCOL_PWM
};

class EscDriverClass
{
  public:
    EscDriverClass();

    int begin(EscProtocol protocol, bool async, int16_t rate) { return 1; }
    //int update()

    int attach(size_t slot_id, int pin, int pulse)
    {
      return 1;
    }

    int write(size_t slot_id, int pulse) ICACHE_RAM_ATTR
    {
      return 1;
    }

    void apply() ICACHE_RAM_ATTR
    {
      commit();
      trigger();
    }

    void commit() ICACHE_RAM_ATTR {}
    //void handle() ICACHE_RAM_ATTR {}
    void trigger() ICACHE_RAM_ATTR {}
};

extern EscDriverClass ESCDriver;

#endif
