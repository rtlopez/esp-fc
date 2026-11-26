#include "GyroMPU6500.hpp"

namespace Espfc::Device::Gyro {

constexpr uint8_t MPU6050_RA_WHO_AM_I = 0x75;
constexpr uint8_t MPU6500_ACCEL_CONF2 = 0x1D;
constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE = 0x70;
constexpr uint8_t MPU6500_WHOAMI_ALT_VALUE = 0x75;

GyroDeviceType GyroMPU6500::getType() const
{
  return GYRO_MPU6500;
}

void GyroMPU6500::setDLPFMode(uint8_t mode)
{
  GyroMPU6050::setDLPFMode(mode);
  _bus->writeByte(_addr, MPU6500_ACCEL_CONF2, mode);
}

bool GyroMPU6500::testConnection()
{
  uint8_t whoami = 0;
  if (_bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami) != 1) return false;
  setChipId(whoami);
  return whoami == MPU6500_WHOAMI_DEFAULT_VALUE || whoami == MPU6500_WHOAMI_ALT_VALUE;
}

} // namespace Espfc::Device::Gyro
