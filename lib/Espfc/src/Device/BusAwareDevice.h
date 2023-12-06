#ifndef _ESPFC_DEVICE_BUS_AWARE_DEVICE_H_
#define _ESPFC_DEVICE_BUS_AWARE_DEVICE_H_

#include "BusDevice.h"

namespace Espfc {

namespace Device {

class BusAwareDevice
{
  public:
    void setBus(BusDevice * bus, uint8_t addr)
    {
      _bus = bus;
      _addr = addr;
    }
  
    const BusDevice * getBus() const
    {
      return _bus;
    }

    uint8_t getAddress() const
    {
      return _addr;
    }

  protected:
    BusDevice * _bus;
    uint8_t _addr;
};

}

}

#endif