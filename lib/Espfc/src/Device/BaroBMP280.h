#ifndef _ESPFC_DEVICE_BARO_BMP280_H_
#define _ESPFC_DEVICE_BARO_BMP280_H_

#include "BusDevice.h"
#include "BaroDevice.h"
#include "Debug_Espfc.h"

#define BMP280_DEFAULT_ADDRESS        0x77
#define BMP280_WHOAMI_ID              0x58

#define BMP280_WHOAMI_REG             0xD0
#define BMP280_VERSION_REG            0xD1
#define BMP280_RESET_REG              0xE0

#define BMP280_CALIB_REG              0x88
#define BMP280_CAL26_REG              0xE1     // R calibration stored in 0xE1-0xF0

#define BMP280_CONTROL_REG            0xF4
#define BMP280_CONFIG_REG             0xF5
#define BMP280_PRESSURE_REG           0xF7
#define BMP280_TEMPERATURE_REG        0xFA

namespace Espfc {

namespace Device {

class BaroBMP280: public BaroDevice
{
  public:
    struct CalibrationData {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;
      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;
      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    };

    int begin(BusDevice * bus) override
    {
      return begin(bus, BMP280_DEFAULT_ADDRESS);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      _bus->writeByte(_addr, BMP280_RESET_REG, 0xB6); // device reset
      delay(10);

      _bus->read(_addr, BMP280_CALIB_REG, sizeof(CalibrationData), (uint8_t*)&_cal);

      uint8_t conf = 0;
      conf |= (1 << 5); // osps_t: x1
      conf |= (1 << 2); // osps_p: x1
      conf |= 3;        // mode: normal;
      _bus->writeByte(_addr, BMP280_CONTROL_REG, conf);

      return 1;
    }

    BaroDeviceType getType() const override
    {
      return BARO_BMP280;
    }

    virtual float readTemperature() override
    {
      int32_t adc_T = readReg(BMP280_TEMPERATURE_REG);
      adc_T >>= 4;

      int32_t var1 = ((((adc_T >> 3) - ((int32_t)_cal.dig_T1 << 1))) * ((int32_t)_cal.dig_T2)) >> 11;
      int32_t var2 = (((((adc_T >> 4) - ((int32_t)_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)_cal.dig_T1))) >> 12) * ((int32_t)_cal.dig_T3)) >> 14;
      _t_fine = var1 + var2;

      float T = (_t_fine * 5 + 128) >> 8;
      return T * 0.01f;
    }

    virtual float readPressure() override
    {
      int32_t adc_P = readReg(BMP280_PRESSURE_REG);
      adc_P >>= 4;

      int64_t var1 = ((int64_t)_t_fine) - 128000;
      int64_t var2 = var1 * var1 * (int64_t)_cal.dig_P6;
      var2 += ((var1*(int64_t)_cal.dig_P5) << 17);
      var2 += (((int64_t)_cal.dig_P4) << 35);
      var1 = ((var1 * var1 * (int64_t)_cal.dig_P3) >> 8) + ((var1 * (int64_t)_cal.dig_P2) << 12);
      var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_cal.dig_P1) >> 33;

      if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
      }
      int64_t p = 1048576 - adc_P;
      p = (((p << 31) - var2) * 3125) / var1;
      var1 = (((int64_t)_cal.dig_P9) * (p>>13) * (p>>13)) >> 25;
      var2 = (((int64_t)_cal.dig_P8) * p) >> 19;

      p = ((p + var1 + var2) >> 8) + (((int64_t)_cal.dig_P7) << 4);

      return (float)p / 256;
    }

    void setMode(BaroDeviceMode mode)
    {
      (void)mode;
    }

    virtual int getDelay() const override
    {
      return 5500;
      //const int comp = 50;
      //return 4500 + comp;        // temp
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, BMP280_WHOAMI_REG, &whoami);
      return whoami == BMP280_WHOAMI_ID;
    }

  protected:
    int32_t readReg(uint8_t reg)
    {
      uint8_t buffer[3];
      _bus->read(_addr, reg, 3, buffer);
      return buffer[2] | (buffer[1] << 8) | (buffer[0] << 16);
    }

    int8_t _mode;
    int32_t _t_fine;
    CalibrationData _cal;
};

}

}

#endif
