#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroSPL06 : public BaroDevice
{
public:
  struct CalibrationData
  {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
  } __attribute__((__packed__));

  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  BaroDeviceType getType() const final;

  float readTemperature() final;
  float readPressure() final;

  void setMode(BaroDeviceMode mode);
  virtual int getDelay(BaroDeviceMode mode) const final;

  bool testConnection() final;

protected:
  void readMesurment();
  int8_t writeReg(uint8_t reg, uint8_t val);

  int8_t _mode;
  int32_t _t_fine;
  int32_t _raw_temp;
  int32_t _raw_pressure;
  CalibrationData _cal;
  int32_t kp;
  int32_t kt;
};

} // namespace Espfc::Device::Baro
