#include "GyroICM20602.hpp"

#define MPU6050_RA_WHO_AM_I           0x75
#define ICM20602_RA_ACCEL2_CONFIG     0x1D
#define ICM20602_WHOAMI_DEFAULT_VALUE 0x12

namespace Espfc::Device::Gyro {

GyroDeviceType GyroICM20602::getType() const
{
  return GYRO_ICM20602;
}

void GyroICM20602::setDLPFMode(uint8_t mode)
{
  GyroMPU6050::setDLPFMode(mode);
  _bus->writeByte(_addr, ICM20602_RA_ACCEL2_CONFIG, mode);
}

bool GyroICM20602::testConnection()
{
  uint8_t whoami = 0;
  _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
  return whoami == ICM20602_WHOAMI_DEFAULT_VALUE;
}

} // namespace Espfc::Device::Gyro
