#ifndef _ESPFC_DEVICE_MAG_QMC5338L_H_
#define _ESPFC_DEVICE_MAG_QMC5338L_H_

#include "MagDevice.h"
#include "BusDevice.h"

#define QMC5883L_ADDRESS            0x0D // this device only has one address
#define QMC5883L_DEFAULT_ADDRESS    0x0D

#define QMC5883L_RA_CONFIG_A        0x09
#define QMC5883L_RA_CONFIG_B        0x0A
#define QMC5883L_RA_MODE            0x09

#define QMC5883L_RA_STATUS          0x06

#define QMC5883L_RA_DATAX_H         0x01
#define QMC5883L_RA_DATAX_L         0x00
#define QMC5883L_RA_DATAZ_H         0x05
#define QMC5883L_RA_DATAZ_L         0x04
#define QMC5883L_RA_DATAY_H         0x03
#define QMC5883L_RA_DATAY_L         0x02

#define QMC5883L_RA_TEMP_L          0x07
#define QMC5883L_RA_TEMP_H          0x08

#define QMC5883L_RA_RST_TIME        0x0B
#define QMC5883L_RA_RSV             0x0C
#define QMC5883L_RA_CHIPID          0x0D

#define QMC5883L_RATE_10            0x00
#define QMC5883L_RATE_50            0x01
#define QMC5883L_RATE_100           0x02
#define QMC5883L_RATE_200           0x03

#define QMC5883L_RANGE_2G           0x00
#define QMC5883L_RANGE_8G           0x01

#define QMC5883L_OSR_512            0x00
#define QMC5883L_OSR_256            0x01
#define QMC5883L_OSR_128            0x02
#define QMC5883L_OSR_64             0x03

#define QMC5883L_MODE_CONTINUOUS    0x01
#define QMC5883L_MODE_IDLE          0x00

#define QMC5883L_STATUS_LOCK_BIT    1
#define QMC5883L_STATUS_READY_BIT   0


namespace Espfc {

namespace Device {

class MagQMC5338L: public MagDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, QMC5883L_DEFAULT_ADDRESS);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      setMode(QMC5883L_OSR_64, QMC5883L_RANGE_2G, QMC5883L_RATE_100, QMC5883L_MODE_CONTINUOUS);

      uint8_t buffer[6];
      _bus->read(_addr, QMC5883L_RA_DATAX_H, 6, buffer);

      return 1;
    }

    int readMag(VectorInt16& v) override
    {
      uint8_t buffer[6];
      _bus->readFast(_addr, QMC5883L_RA_DATAX_L, 6, buffer);

      v.x = (((int16_t)buffer[1]) << 8) | buffer[0];

      v.y = (((int16_t)buffer[3]) << 8) | buffer[2];

      v.z = (((int16_t)buffer[5]) << 8) | buffer[4];

      return 1;
    }

    const VectorFloat convert(const VectorInt16& v) const override
    {
      const float scale = 1.f / 1090.f;
      return VectorFloat{v} * scale;
    }

    int getRate() const override
    {
      return 75;
    }

    virtual MagDeviceType getType() const override
    {
      return MAG_QMC5883;
    }

    void setMode(uint8_t osr, uint8_t rng, uint8_t odr, uint8_t mode)
    {
      _mode = (osr << 6) | (rng << 4) | (odr << 2) | (mode);
      uint8_t res = _bus->writeByte(_addr, QMC5883L_RA_MODE, ((osr << 6) | (rng << 4) | (odr << 2) | (mode)));

      (void)res;
    }

    bool testConnection() override
    {
      uint8_t buffer[3] = { 0, 0, 0 };
      uint8_t len = _bus->read(_addr, QMC5883L_RA_CHIPID, 1, buffer);

      return len == 1 && buffer[0] == 0xFF;
    }

  private:
    uint8_t _mode;
};

}

}

#endif
