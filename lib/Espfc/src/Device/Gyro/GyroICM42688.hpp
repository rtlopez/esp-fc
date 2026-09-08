#pragma once

#include "Device/GyroDevice.hpp"
#include "helper_3dmath.hpp"

namespace Espfc::Device::Gyro {

class GyroICM42688 : public GyroDevice
{
public:
  int begin(BusDevice* bus) override;
  int begin(BusDevice* bus, uint8_t addr) override;

  GyroDeviceType getType() const override;

  int readGyro(VectorInt16& v) override;
  int readAccel(VectorInt16& v) override;

  // ICM-42688-P AAF lives in Bank 1 registers; no-op matches GyroBMI160 pattern
  void setDLPFMode(uint8_t mode) override;

  int getRate() const override;

  void setRate(int rate) override;

  bool testConnection() override;
};

} // namespace Espfc::Device::Gyro
