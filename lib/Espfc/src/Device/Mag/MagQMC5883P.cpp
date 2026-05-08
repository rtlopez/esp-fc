#include "MagQMC5883P.hpp"
#include <Arduino.h>

#define QMC5883P_ADDRESS 0x2C
#define QMC5883P_DEFAULT_ADDRESS 0x2C
#define QMC5883P_CHIP_ID 0x80

#define QMC5883P_REG_CHIPID 0x00
#define QMC5883P_REG_XOUT_LSB 0x01
#define QMC5883P_REG_XOUT_MSB 0x02
#define QMC5883P_REG_YOUT_LSB 0x03
#define QMC5883P_REG_YOUT_MSB 0x04
#define QMC5883P_REG_ZOUT_LSB 0x05
#define QMC5883P_REG_ZOUT_MSB 0x06
#define QMC5883P_REG_STATUS 0x09
#define QMC5883P_REG_CONTROL1 0x0A
#define QMC5883P_REG_CONTROL2 0x0B

#define QMC5883P_MODE_SUSPEND 0x00
#define QMC5883P_MODE_NORMAL 0x01
#define QMC5883P_MODE_SINGLE 0x02
#define QMC5883P_MODE_CONTINUOUS 0x03

#define QMC5883P_ODR_10HZ 0x00
#define QMC5883P_ODR_50HZ 0x01
#define QMC5883P_ODR_100HZ 0x02
#define QMC5883P_ODR_200HZ 0x03

#define QMC5883P_OSR_8 0x00
#define QMC5883P_OSR_4 0x01
#define QMC5883P_OSR_2 0x02
#define QMC5883P_OSR_1 0x03

#define QMC5883P_DSR_1 0x00
#define QMC5883P_DSR_2 0x01
#define QMC5883P_DSR_4 0x02
#define QMC5883P_DSR_8 0x03

#define QMC5883P_RANGE_30G 0x00
#define QMC5883P_RANGE_12G 0x01
#define QMC5883P_RANGE_8G 0x02
#define QMC5883P_RANGE_2G 0x03

#define QMC5883P_SETRESET_ON 0x00
#define QMC5883P_SETRESET_SETONLY 0x01
#define QMC5883P_SETRESET_OFF 0x02

#define QMC5883P_CONTROL2_RESET_BIT 7
#define QMC5883P_RESET_DELAY_MS 50

namespace Espfc::Device::Mag {

MagQMC5883P::MagQMC5883P(): _currentRange{QMC5883P_RANGE_8G}, _currentOdr{QMC5883P_ODR_100HZ} {}

int MagQMC5883P::begin(BusDevice* bus)
{
  return begin(bus, QMC5883P_DEFAULT_ADDRESS);
}

int MagQMC5883P::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  _currentRange = QMC5883P_RANGE_8G;
  _currentOdr = QMC5883P_ODR_100HZ;

  delay(2);
  if (!testConnection()) return 0;
  if (!softReset()) return 0;

  const uint8_t ctrl2 = makeControl2(_currentRange, QMC5883P_SETRESET_ON);
  if (!_bus->writeByte(_addr, QMC5883P_REG_CONTROL2, ctrl2)) return 0;

  const uint8_t ctrl1 = makeControl1(QMC5883P_MODE_CONTINUOUS, _currentOdr, QMC5883P_OSR_1, QMC5883P_DSR_1);
  if (!_bus->writeByte(_addr, QMC5883P_REG_CONTROL1, ctrl1)) return 0;

  delay(10);

  // Prime the external sensor read window when the magnetometer is behind a master device.
  if (_bus->read(_addr, QMC5883P_REG_XOUT_LSB, 6, _buffer) != 6) return 0;

  return 1;
}

int MagQMC5883P::readMag(VectorInt16& v)
{
  if (_bus->readFast(_addr, QMC5883P_REG_XOUT_LSB, 6, _buffer) != 6) return 0;

  v.x = (((int16_t)_buffer[1]) << 8) | _buffer[0];
  v.y = (((int16_t)_buffer[3]) << 8) | _buffer[2];
  v.z = (((int16_t)_buffer[5]) << 8) | _buffer[4];

  return 1;
}

const VectorFloat MagQMC5883P::convert(const VectorInt16& v) const
{
  float lsbPerGauss = 3750.0f;
  switch (_currentRange)
  {
    case QMC5883P_RANGE_30G: lsbPerGauss = 1000.0f; break;
    case QMC5883P_RANGE_12G: lsbPerGauss = 2500.0f; break;
    case QMC5883P_RANGE_8G: lsbPerGauss = 3750.0f; break;
    case QMC5883P_RANGE_2G: lsbPerGauss = 15000.0f; break;
  }

  return static_cast<VectorFloat>(v) * (1.0f / lsbPerGauss);
}

int MagQMC5883P::getRate() const
{
  switch (_currentOdr)
  {
    case QMC5883P_ODR_10HZ: return 10;
    case QMC5883P_ODR_50HZ: return 50;
    case QMC5883P_ODR_200HZ: return 200;
    case QMC5883P_ODR_100HZ:
    default: return 100;
  }
}

MagDeviceType MagQMC5883P::getType() const
{
  return MAG_QMC5883P;
}

bool MagQMC5883P::softReset()
{
  if (!_bus->writeByte(_addr, QMC5883P_REG_CONTROL2, (1u << QMC5883P_CONTROL2_RESET_BIT))) return false;
  delay(QMC5883P_RESET_DELAY_MS);
  return testConnection();
}

bool MagQMC5883P::testConnection()
{
  uint8_t chipId = 0;
  for (uint8_t attempt = 0; attempt < 3; attempt++)
  {
    if (_bus->read(_addr, QMC5883P_REG_CHIPID, 1, &chipId) == 1 && chipId == QMC5883P_CHIP_ID)
    {
      return true;
    }
    delay(2);
  }

  return false;
}

uint8_t MagQMC5883P::makeControl1(uint8_t mode, uint8_t odr, uint8_t osr, uint8_t dsr)
{
  return (mode & 0x03) | ((odr & 0x03) << 2) | ((osr & 0x03) << 4) | ((dsr & 0x03) << 6);
}

uint8_t MagQMC5883P::makeControl2(uint8_t range, uint8_t setReset)
{
  return ((range & 0x03) << 2) | (setReset & 0x03);
}

} // namespace Espfc::Device::Mag
