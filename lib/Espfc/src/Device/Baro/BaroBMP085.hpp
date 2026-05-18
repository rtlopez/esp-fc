#pragma once

#include "Debug_Espfc.h"
#include "Device/BaroDevice.hpp"

namespace Espfc::Device::Baro {

class BaroBMP085 : public BaroDevice
{
public:
  struct CalibrationData
  {
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
  int8_t _mode;
  int32_t _t_fine;
  CalibrationData _cal;
};

} // namespace Espfc::Device::Baro
