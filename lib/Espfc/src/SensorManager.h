#pragma once

#include "Model.h"
#include "Control/Fusion.h"
#include "Control/Altitude.hpp"
#include "Sensor/GyroSensor.h"
#include "Sensor/AccelSensor.h"
#include "Sensor/MagSensor.h"
#include "Sensor/BaroSensor.h"
#include "Sensor/VoltageSensor.h"

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
};

}
