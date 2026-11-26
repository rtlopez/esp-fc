#pragma once

#include "Device/MagDevice.hpp"
#include "Hal/BusDevice.hpp"

namespace Espfc::Device::Mag {

class MagHMC5883L : public MagDevice
{
public:
  int begin(Hal::BusDevice* bus) final;
  int begin(Hal::BusDevice* bus, uint8_t addr) final;

  MagDeviceType getType() const final;
  int getRate() const final;

  int readMag(VectorInt16& v) final;
  const VectorFloat convert(const VectorInt16& v) const final;

  bool testConnection() final;
};

} // namespace Espfc::Device::Mag
