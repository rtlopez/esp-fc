#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroBMP280 : public BaroDevice
{
public:
  struct CalibrationData
  {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    /*uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;*/
  } __attribute__((__packed__));

  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode);
  int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

protected:
  void readMesurment();

  int8_t writeReg(uint8_t reg, uint8_t val);

  int8_t _mode;
  int32_t _t_fine;
  int32_t _raw_temp;
  int32_t _raw_pressure;
  CalibrationData _cal;
};

} // namespace Espfc::Device::Baro
