#ifndef _ESPFC_DEVICE_BUS_AWARE_DEVICE_H_
#define _ESPFC_DEVICE_BUS_AWARE_DEVICE_H_

#include "BusDevice.h"

namespace Espfc {

namespace Device {

class BusAwareDevice
{
  public:
    void setBus(BusDevice * bus, uint8_t addr, uint8_t masterAddr = 0)
    {
      _bus = bus;
      _addr = addr;
      _masterAddr = masterAddr;
    }
  
    const BusDevice * getBus() const
    {
      return _bus;
    }

  protected:
    BusDevice * _bus;
    uint8_t _addr;
    uint8_t _masterAddr;
};

}

}

#endif