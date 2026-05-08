#pragma once

#include "Device/BusDevice.hpp"
#include "Device/MagDevice.hpp"

namespace Espfc::Device::Mag {

class MagAK8963 : public MagDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  int readMag(VectorInt16& v) final;
  const VectorFloat convert(const VectorInt16& v) const final;

  int getRate() const final;
  MagDeviceType getType() const final;

  bool testConnection() final;

private:
  uint8_t _mode;
  VectorFloat _scale;
  uint8_t _buffer[7];
};

} // namespace Espfc::Device::Mag
