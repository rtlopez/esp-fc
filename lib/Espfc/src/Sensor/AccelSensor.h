#pragma once

#include "BaseSensor.h"
#include "Device/GyroDevice.h"
#include "Model.h"

namespace Espfc::Sensor {

class AccelSensor : public BaseSensor
{
public:
  AccelSensor(Model& model);

  int begin();
  int update();
  int read();
  int filter();
  void updateTrimRotation();

private:
  void calibrate();

  Model& _model;
  Device::GyroDevice* _gyro;
  Utils::Filter _filter[AXIS_COUNT_RPY];
};

} // namespace Espfc::Sensor
