#pragma once

#include "Model.h"
#include "Fusion.h"
#include "Sensor/GyroSensor.h"
#include "Sensor/AccelSensor.h"
#include "Sensor/MagSensor.h"
#include "Sensor/BaroSensor.h"
#include "Sensor/VoltageSensor.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

class SensorManager
{
  public:
    SensorManager(Model& model);

    int begin();
    int read() FAST_CODE_ATTR;
    int preLoop() FAST_CODE_ATTR;
    int postLoop();
    int fusion();
    // main task
    int update() FAST_CODE_ATTR;
    // sub task
    int updateDelayed();

  private:
    Model& _model;
    Sensor::GyroSensor _gyro;
    Sensor::AccelSensor _accel;
    Sensor::MagSensor _mag;
    Sensor::BaroSensor _baro;
    Sensor::VoltageSensor _voltage;
    Fusion _fusion;
    bool _fusionUpdate;
};

}
