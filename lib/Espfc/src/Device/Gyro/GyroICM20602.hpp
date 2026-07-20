#pragma once

#include "GyroMPU6050.hpp"

namespace Espfc::Device::Gyro {

class GyroICM20602: public GyroMPU6050
{
  public:
    GyroDeviceType getType() const override;

    void setDLPFMode(uint8_t mode) override;

    bool testConnection() override;
};

} // namespace Espfc::Device::Gyro
