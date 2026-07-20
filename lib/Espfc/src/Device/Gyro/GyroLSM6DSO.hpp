#pragma once

#include "Device/GyroDevice.hpp"
#include "helper_3dmath.hpp"

namespace Espfc::Device::Gyro {

class GyroLSM6DSO: public GyroDevice
{
  public:
    int begin(BusDevice * bus) override;
    int begin(BusDevice * bus, uint8_t addr) override;

    GyroDeviceType getType() const override;

    int readGyro(VectorInt16& v) override;
    int readAccel(VectorInt16& v) override;

    void setDLPFMode(uint8_t mode) override;

    int getRate() const override;

    void setRate(int rate) override;

    bool testConnection() override;
};

} // namespace Espfc::Device::Gyro
