#include "GyroICM42670P.hpp"

#include "Hal/FastCode.hpp"
#include "Hal/Time.hpp"

#define ICM42670P_ADDRESS_FIRST 0x68
#define ICM42670P_ADDRESS_SECOND 0x69

#define ICM42670P_REG_SIGNAL_PATH_RESET 0x02
#define ICM42670P_REG_ACCEL_DATA_X1 0x0B
#define ICM42670P_REG_GYRO_DATA_X1 0x11
#define ICM42670P_REG_PWR_MGMT0 0x1F
#define ICM42670P_REG_GYRO_CONFIG0 0x20
#define ICM42670P_REG_ACCEL_CONFIG0 0x21
#define ICM42670P_REG_WHO_AM_I 0x75

#define ICM42670P_WHO_AM_I_VALUE 0x67
#define ICM42670P_SOFT_RESET 0x10
#define ICM42670P_PWR_GYRO_ACCEL_LN 0x0F
#define ICM42670P_GYRO_2000DPS_1600HZ 0x05
#define ICM42670P_ACCEL_16G_1600HZ 0x05

namespace Espfc::Device::Gyro {

int GyroICM42670P::begin(Hal::BusDevice* bus)
{
  return begin(bus, ICM42670P_ADDRESS_FIRST) ? 1 : begin(bus, ICM42670P_ADDRESS_SECOND);
}

int GyroICM42670P::begin(Hal::BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  _bus->writeByte(_addr, ICM42670P_REG_SIGNAL_PATH_RESET, ICM42670P_SOFT_RESET);
  delay(2); // no check as min 1ms after soft reset is required before register access (see datasheet)

  if (!_bus->writeByte(_addr, ICM42670P_REG_PWR_MGMT0, ICM42670P_PWR_GYRO_ACCEL_LN)) return 0;
  delay(1);

  if (!_bus->writeByte(_addr, ICM42670P_REG_GYRO_CONFIG0, ICM42670P_GYRO_2000DPS_1600HZ)) return 0;
  if (!_bus->writeByte(_addr, ICM42670P_REG_ACCEL_CONFIG0, ICM42670P_ACCEL_16G_1600HZ)) return 0;

  return 1;
}

GyroDeviceType GyroICM42670P::getType() const
{
  return GYRO_ICM42670P;
}

int FAST_CODE_ATTR GyroICM42670P::readGyro(VectorInt16& v)
{
  uint8_t buffer[6] = {};
  if (_bus->readFast(_addr, ICM42670P_REG_GYRO_DATA_X1, 6, buffer) != 6) return 0;

  v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
  v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
  v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

  return 1;
}

int FAST_CODE_ATTR GyroICM42670P::readAccel(VectorInt16& v)
{
  uint8_t buffer[6] = {};
  if (_bus->readFast(_addr, ICM42670P_REG_ACCEL_DATA_X1, 6, buffer) != 6) return 0;

  v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
  v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
  v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

  return 1;
}

void GyroICM42670P::setDLPFMode(uint8_t mode) {}

int GyroICM42670P::getRate() const
{
  return 1600;
}

void GyroICM42670P::setRate(int rate) {}

bool GyroICM42670P::testConnection()
{
  uint8_t whoami = 0;
  if (_bus->readByte(_addr, ICM42670P_REG_WHO_AM_I, &whoami) != 1) return false;
  setChipId(whoami);
  return whoami == ICM42670P_WHO_AM_I_VALUE;
}

} // namespace Espfc::Device::Gyro
