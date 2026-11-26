#include "GyroICM20602.hpp"

namespace Espfc::Device::Gyro {

constexpr uint8_t MPU6050_RA_WHO_AM_I = 0x75;
constexpr uint8_t ICM20602_RA_ACCEL2_CONFIG = 0x1D;
constexpr uint8_t ICM20602_WHOAMI_DEFAULT_VALUE = 0x12;

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
  if (_bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami) != 1) return false;
  setChipId(whoami);
  return whoami == ICM20602_WHOAMI_DEFAULT_VALUE;
}

} // namespace Espfc::Device::Gyro
