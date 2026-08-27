#pragma once

#include "Device/BusDevice.hpp"
#include <optional>

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

  std::optional<uint8_t> getChipId() const
  {
    return _chipId;
  }

protected:
  void setChipId(uint8_t chipId)
  {
    _chipId = chipId;
  }

  BusDevice* _bus = nullptr;
  uint8_t _addr = 0;
  std::optional<uint8_t> _chipId = {};
};

} // namespace Espfc::Device
