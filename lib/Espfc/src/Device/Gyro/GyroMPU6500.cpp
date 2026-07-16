#include "GyroMPU6500.hpp"

#define MPU6050_RA_WHO_AM_I          0x75
#define MPU6500_ACCEL_CONF2          0x1D
#define MPU6500_WHOAMI_DEFAULT_VALUE 0x70
#define MPU6500_WHOAMI_ALT_VALUE     0x75

namespace Espfc::Device::Gyro {

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
  uint8_t len = _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
  return len == 1 && (whoami == MPU6500_WHOAMI_DEFAULT_VALUE || whoami == MPU6500_WHOAMI_ALT_VALUE);
}

} // namespace Espfc::Device::Gyro
