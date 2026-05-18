#pragma once

#include "BaseSensor.h"
#include "Device/MagDevice.hpp"
#include "Model.h"

namespace Espfc::Sensor {

class MagSensor : public BaseSensor
{
public:
  MagSensor(Model& model);

  int begin();
  int update();
  int read();
  int filter();

private:
  void calibrate();
  void resetCalibration();
  void updateCalibration();
  void applyCalibration();

  Model& _model;
  Device::MagDevice* _mag;
};

} // namespace Espfc::Sensor
