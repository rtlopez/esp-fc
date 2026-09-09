#pragma once

#include "Hal/BusDevice.hpp"
#include "Device/MagDevice.hpp"

namespace Espfc::Device::Mag {

class MagQMC5883L : public MagDevice
{
public:
  int begin(Hal::BusDevice* bus) final;
  int begin(Hal::BusDevice* bus, uint8_t addr) final;

  int readMag(VectorInt16& v) final;
  const VectorFloat convert(const VectorInt16& v) const final;

  int getRate() const final;
  MagDeviceType getType() const final;

  bool testConnection() final;

private:
  void setMode(uint8_t osr, uint8_t rng, uint8_t odr, uint8_t mode);

  uint8_t _mode;
};

} // namespace Espfc::Device::Mag
