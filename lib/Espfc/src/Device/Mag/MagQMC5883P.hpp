#pragma once

#include "Device/BusDevice.hpp"
#include "Device/MagDevice.hpp"

namespace Espfc::Device::Mag {

class MagQMC5883P : public MagDevice
{
public:
  MagQMC5883P();

  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  int readMag(VectorInt16& v) final;
  const VectorFloat convert(const VectorInt16& v) const final;

  int getRate() const final;
  MagDeviceType getType() const final;
  bool testConnection() final;

private:
  bool softReset();
  static uint8_t makeControl1(uint8_t mode, uint8_t odr, uint8_t osr, uint8_t dsr);
  static uint8_t makeControl2(uint8_t range, uint8_t setReset);

  uint8_t _currentRange;
  uint8_t _currentOdr;
  uint8_t _buffer[6] = {0};
};

} // namespace Espfc::Device::Mag
