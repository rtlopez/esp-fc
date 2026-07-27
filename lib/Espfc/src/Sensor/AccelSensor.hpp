#pragma once

#include "Device/GyroDevice.hpp"
#include "Model.h"
#include "Sensor/BaseSensor.hpp"

namespace Espfc::Sensor {

class AccelSensor : public BaseSensor
{
public:
  AccelSensor(Model& model);

  int begin();
  int update();
  int read();
  int filter();

private:
  void calibrate(VectorFloat& accel);

  Model& _model;
  Device::GyroDevice* _gyro;
  Utils::Filter _filter[AXIS_COUNT_RPY];
};

} // namespace Espfc::Sensor
