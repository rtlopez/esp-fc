#include "GyroICM42688.hpp"

#include "Debug_Espfc.h"
#include "Utils/MemoryHelper.h"

#define ICM42688_REG_WHO_AM_I         0x75
#define ICM42688_REG_DEVICE_CONFIG    0x11
#define ICM42688_REG_PWR_MGMT0        0x4E
#define ICM42688_REG_GYRO_CONFIG0     0x4F
#define ICM42688_REG_ACCEL_CONFIG0    0x50
#define ICM42688_REG_ACCEL_DATA_X1    0x1F
#define ICM42688_REG_GYRO_DATA_X1     0x25

#define ICM42688_WHO_AM_I_VALUE       0x47
#define ICM42688_SOFT_RESET           0x01
#define ICM42688_PWR_GYRO_ACCEL_LN    0x0F
#define ICM42688_GYRO_2000DPS_8KHZ    0x03
#define ICM42688_ACCEL_16G_8KHZ       0x03

namespace Espfc::Device::Gyro {

int GyroICM42688::begin(BusDevice* bus)
{
  return begin(bus, 0);
}

int GyroICM42688::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  _bus->writeByte(_addr, ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET);
  delay(2); // no check as min 1ms after soft reset is required before register access (see datasheet)

  if (!_bus->writeByte(_addr, ICM42688_REG_PWR_MGMT0, ICM42688_PWR_GYRO_ACCEL_LN)) return 0;
  delay(1);

  if (!_bus->writeByte(_addr, ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_2000DPS_8KHZ)) return 0;
  if (!_bus->writeByte(_addr, ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_16G_8KHZ)) return 0;

  return 1;
}

GyroDeviceType GyroICM42688::getType() const
{
  return GYRO_ICM42688;
}

int FAST_CODE_ATTR GyroICM42688::readGyro(VectorInt16& v)
{
  uint8_t buffer[6];
  _bus->readFast(_addr, ICM42688_REG_GYRO_DATA_X1, 6, buffer);
  v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
  v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
  v.z = (((int16_t)buffer[4]) << 8) | buffer[5];
  return 1;
}

int FAST_CODE_ATTR GyroICM42688::readAccel(VectorInt16& v)
{
  uint8_t buffer[6];
  _bus->readFast(_addr, ICM42688_REG_ACCEL_DATA_X1, 6, buffer);
  v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
  v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
  v.z = (((int16_t)buffer[4]) << 8) | buffer[5];
  return 1;
}

// ICM-42688-P AAF lives in Bank 1 registers; no-op matches GyroBMI160 pattern
void GyroICM42688::setDLPFMode(uint8_t mode) {}

int GyroICM42688::getRate() const
{
  return 8000;
}

void GyroICM42688::setRate(int rate) {}

bool GyroICM42688::testConnection()
{
  uint8_t whoami = 0;
  _bus->readByte(_addr, ICM42688_REG_WHO_AM_I, &whoami);
  // D("icm42688:whoami", _addr, whoami);
  return whoami == ICM42688_WHO_AM_I_VALUE;
}

} // namespace Espfc::Device::Gyro
