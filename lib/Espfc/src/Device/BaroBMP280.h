#ifndef _ESPFC_DEVICE_BARO_BMP280_H_
#define _ESPFC_DEVICE_BARO_BMP280_H_

#include "BusDevice.h"
#include "BaroDevice.h"
#include "Debug_Espfc.h"

#define BMP280_ADDRESS_FIRST          0x76
#define BMP280_ADDRESS_SECOND         0x77
#define BMP280_WHOAMI_ID              0x58

#define BMP280_WHOAMI_REG             0xD0
#define BMP280_VERSION_REG            0xD1
#define BMP280_RESET_REG              0xE0
#define BMP280_RESET_VAL              0xB6

#define BMP280_CALIB_REG              0x88
#define BMP280_CAL26_REG              0xE1     // R calibration stored in 0xE1-0xF0

#define BMP280_STATUS_REG             0xF3
#define BMP280_CONTROL_REG            0xF4
#define BMP280_CONFIG_REG             0xF5
#define BMP280_PRESSURE_REG           0xF7
#define BMP280_TEMPERATURE_REG        0xFA

#define BMP280_FILTER_X2              (1 << 2)
#define BMP280_MODE_NORMAL            0x03

#define BMP280_SAMPLING_X1            1
#define BMP280_SAMPLING_X2            2
#define BMP280_SAMPLING_X4            3
#define BMP280_SAMPLING_X8            4

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
      /*uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;*/
    } __attribute__ ((__packed__));

    int begin(BusDevice * bus) override
    {
      return begin(bus, BMP280_ADDRESS_FIRST) ? 1 : begin(bus, BMP280_ADDRESS_SECOND) ? 1 : 0;
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      _bus->read(_addr, BMP280_CALIB_REG, sizeof(CalibrationData), (uint8_t*)&_cal); // read callibration

      writeReg(BMP280_RESET_REG, BMP280_RESET_VAL); // device reset
      delay(2);

      writeReg(BMP280_CONFIG_REG, BMP280_FILTER_X2); // set minimal standby and IIR filter X2
      //writeReg(BMP280_CONFIG_REG, 0); // set minimal standby and IIR filter off

      writeReg(BMP280_CONTROL_REG, BMP280_SAMPLING_X1 << 5 | BMP280_SAMPLING_X4 << 2 | BMP280_MODE_NORMAL); // set sampling mode

      delay(20);

      return 1;
    }

    BaroDeviceType getType() const override
    {
      return BARO_BMP280;
    }

    virtual float readTemperature() override
    {
      float T = (_t_fine * 5 + 128) >> 8;
      return T * 0.01f;
    }

    virtual float readPressure() override
    {
      readMesurment();

      int32_t adc_T = _raw_temp;
      adc_T >>= 4;

      int32_t vart1 = ((((adc_T >> 3) - ((int32_t)_cal.dig_T1 << 1))) * ((int32_t)_cal.dig_T2)) >> 11;
      int32_t vart2 = (((((adc_T >> 4) - ((int32_t)_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)_cal.dig_T1))) >> 12) * ((int32_t)_cal.dig_T3)) >> 14;
      _t_fine = vart1 + vart2;
      //_t_fine += ((var1 + var2) - _t_fine + 4) >> 3; // smooth t_fine

      int32_t adc_P = _raw_pressure;
      adc_P >>= 4;

      int64_t var1 = ((int64_t)_t_fine) - 128000;
      int64_t var2 = var1 * var1 * (int64_t)_cal.dig_P6;
      var2 += ((var1 * (int64_t)_cal.dig_P5) << 17);
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

      return ((float)p) / 256.f;
    }

    void setMode(BaroDeviceMode mode)
    {
      (void)mode;
    }

    virtual int getDelay(BaroDeviceMode mode) const override
    {
      switch(mode)
      {
        case BARO_MODE_TEMP:
          return 0;
        default:
          //return 5500; // if sapling X1
          //return 7500; // if sampling X2
          return 11500;  // if sampling X4
      }
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->read(_addr, BMP280_WHOAMI_REG, 1, &whoami);
      return whoami == BMP280_WHOAMI_ID;
    }

  protected:
    void readMesurment()
    {
      uint8_t buffer[6] = {0, 0, 0, 0, 0, 0};
      _bus->readFast(_addr, BMP280_PRESSURE_REG, 6, buffer);
      _raw_pressure = buffer[2] | (buffer[1] << 8) | (buffer[0] << 16);
      _raw_temp = buffer[5] | (buffer[4] << 8) | (buffer[3] << 16);
    }

    int8_t writeReg(uint8_t reg, uint8_t val)
    {
      return _bus->write(_addr, reg, 1, &val);
    }

    int8_t _mode;
    int32_t _t_fine;
    int32_t _raw_temp;
    int32_t _raw_pressure;
    CalibrationData _cal;
};

}

}

#endif
