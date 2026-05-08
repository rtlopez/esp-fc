#pragma once

#include "Device/BusDevice.hpp"

namespace Espfc::Device {

class BusAwareDevice
{
public:
  void setBus(BusDevice* bus, uint8_t addr)
  {
    _bus = bus;
    _addr = addr;
  }

  const BusDevice* getBus() const
  {
    return _bus;
  }

  uint8_t getAddress() const
  {
    return _addr;
  }

protected:
  BusDevice* _bus;
  uint8_t _addr;
};

} // namespace Espfc::Device
