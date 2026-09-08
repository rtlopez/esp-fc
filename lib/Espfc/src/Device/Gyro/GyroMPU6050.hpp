#pragma once

// https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp#L1501
// https://github.com/guywithaview/Arduino-Test/blob/master/GY87/GY87.ino

#include "Device/BusDevice.hpp"
#include "Device/GyroDevice.hpp"
#include "helper_3dmath.hpp"

namespace Espfc::Device::Gyro {

class GyroMPU6050 : public GyroDevice
{
public:
  int begin(BusDevice* bus) override;
  int begin(BusDevice* bus, uint8_t addr) override;

  GyroDeviceType getType() const override;

  int readGyro(VectorInt16& v) override;
  int readAccel(VectorInt16& v) override;

  void setDLPFMode(uint8_t mode) override;
  int getRate() const override;
  void setRate(int rate) override;

  bool testConnection() override;

  uint8_t _dlpf;
};

} // namespace Espfc::Device::Gyro
