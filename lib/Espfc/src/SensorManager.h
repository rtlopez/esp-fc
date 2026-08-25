#pragma once

#include "Control/Altitude.hpp"
#include "Control/Fusion.h"
#include "Device/Input/InputButton.hpp"
#include "Model.h"
#include "Sensor/AccelSensor.hpp"
#include "Sensor/BaroSensor.hpp"
#include "Sensor/GyroSensor.hpp"
#include "Sensor/MagSensor.hpp"
#include "Sensor/VoltageSensor.hpp"

namespace Espfc {

class SensorManager
{
public:
  SensorManager(Model& model);

  int begin();
  int read();
  int preLoop();
  int postLoop();
  int fusion();
  // main task
  int update();
  // sub task
  int updateDelayed();

  int reload(ModelChangeEvent event);

private:
  Model& _model;
  Sensor::GyroSensor _gyro;
  Sensor::AccelSensor _accel;
  Sensor::MagSensor _mag;
  Sensor::BaroSensor _baro;
  Sensor::VoltageSensor _voltage;
  Control::Fusion _fusion;
  Control::Altitude _altitude;
  bool _fusionUpdate;
  Device::Input::InputButton _button;
};

} // namespace Espfc
