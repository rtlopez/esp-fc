#pragma once

#include "Device/BusDevice.hpp"
#include "Device/MagDevice.hpp"

namespace Espfc::Device::Mag {

class MagHMC5883L : public MagDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  MagDeviceType getType() const final;
  int getRate() const final;

  int readMag(VectorInt16& v) final;
  const VectorFloat convert(const VectorInt16& v) const final;

  bool testConnection() final;

private:
  void setSampleAveraging(uint8_t averaging);
  void setSampleRate(uint8_t rate);
  void setMode(uint8_t mode);
  void setGain(uint8_t gain);

private:
  uint8_t _mode;
};

} // namespace Espfc::Device::Mag
