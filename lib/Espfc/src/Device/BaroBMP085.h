#ifndef _ESPFC_DEVICE_BARO_BMP085_H_
#define _ESPFC_DEVICE_BARO_BMP085_H_

#include "BusDevice.h"
#include "BaroDevice.h"
#include "Debug_Espfc.h"

#define BMP085_DEFAULT_ADDRESS 0x77
#define BMP085_WHOAMI_ID       0x55

#define BMP085_CALIB_REG       0xAA
#define BMP085_WHOAMI_REG      0xD0
#define BMP085_RESET_REG       0xE0
#define BMP085_CONTROL_REG     0xF4
#define BMP085_MEASUREMENT_REG 0xF6

#define BMP085_MEASURE_T       0x2E                // temperature measurent
#define BMP085_MEASURE_P0      0x34                // pressure measurement no oversampling
#define BMP085_MEASURE_P1      0x74                // pressure measurement oversampling x2
#define BMP085_MEASURE_P2      0xB4                // pressure measurement oversampling x4
#define BMP085_MEASURE_P3      0xF4                // pressure measurement oversampling x8


namespace Espfc {

namespace Device {

class BaroBMP085: public BaroDevice
{
  public:
    struct CalibrationData {
      int16_t ac1;
      int16_t ac2;
      int16_t ac3;
      uint16_t ac4;
      uint16_t ac5;
      uint16_t ac6;
      int16_t b1;
      int16_t b2;
      int16_t mb;
      int16_t mc;
      int16_t md;
    } __attribute__ ((__packed__));

    int begin(BusDevice * bus) override
    {
      return begin(bus, BMP085_DEFAULT_ADDRESS);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      _bus->writeByte(_addr, BMP085_RESET_REG, 0xB6); // device reset
      delay(10);

      _bus->read(_addr, BMP085_CALIB_REG, sizeof(CalibrationData), (uint8_t*)&_cal);

      return 1;
    }

    BaroDeviceType getType() const override
    {
      return BARO_BMP085;
    }

    virtual float readTemperature() override
    {
      uint8_t buffer[2];
      _bus->read(_addr, BMP085_MEASUREMENT_REG, 2, buffer);

      // calbrate temp
      int32_t ut = ((uint16_t)buffer[0] << 8) + buffer[1];
      if(ut == 0) return NAN;

      int32_t x1 = ((ut - (int32_t)_cal.ac6) * (int32_t)_cal.ac5) >> 15;
      int32_t x2 = ((int32_t)_cal.mc << 11) / (x1 + _cal.md);
      _t_fine = x1 + x2;
      //_t_fine = (_t_fine + x1 + x2 + 1) >> 1; // avg of last two samples
      return (float)((_t_fine + 8) >> 4) * 0.1f;
    }

    virtual float readPressure() override
    {
      uint8_t buffer[3];
      _bus->read(_addr, BMP085_MEASUREMENT_REG, 3, buffer);

      uint8_t oss = (_mode & 0xC0) >> 6;

      uint32_t up = ((uint32_t)buffer[0] << 16) + ((uint16_t)buffer[1] << 8) + buffer[2];
      if(_mode & 0x34) up = up >> (8 - oss);

      if(up == 0) return NAN;

      int32_t b6 = _t_fine - 4000;
      int32_t x1 = ((int32_t)_cal.b2 * ((b6 * b6) >> 12)) >> 11;
      int32_t x2 = ((int32_t)_cal.ac2 * b6) >> 11;
      int32_t x3 = x1 + x2;
      int32_t b3 = ((((int32_t)_cal.ac1 * 4 + x3) << oss) + 2) >> 2;
      x1 = ((int32_t)_cal.ac3 * b6) >> 13;
      x2 = ((int32_t)_cal.b1 * ((b6 * b6) >> 12)) >> 16;
      x3 = ((x1 + x2) + 2) >> 2;
      uint32_t b4 = ((uint32_t)_cal.ac4 * (uint32_t)(x3 + 32768)) >> 15;
      uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> oss);
      int32_t p = 0;
      if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
      } else {
        p = (b7 / b4) << 1;
      }
      x1 = (p >> 8) * (p >> 8);
      x1 = (x1 * 3038) >> 16;
      x2 = (-7357 * p) >> 16;

      return p + ((x1 + x2 + (int32_t)3791) >> 4);
    }

    void setMode(BaroDeviceMode mode)
    {
      _mode = mode == BARO_MODE_TEMP ? BMP085_MEASURE_T : BMP085_MEASURE_P2;
      _bus->writeByte(_addr, BMP085_CONTROL_REG, _mode);
    }

    virtual int getDelay(BaroDeviceMode mode) const override
    {
      switch(mode)
      {
        case BARO_MODE_TEMP:
          return 4550;        // temp
        default:
          //return 4550;  // press_0
          //return 7550;  // press_1
          return 13550; // press_2
          //return 25550; // press_3
      }
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, BMP085_WHOAMI_REG, &whoami);
      return whoami == BMP085_WHOAMI_ID;
    }

  protected:
    int8_t _mode;
    int32_t _t_fine;
    CalibrationData _cal;
};

}

}

#endif
