#pragma once

#include "Model.h"
#include "BaseSensor.h"
#include "Device/GyroDevice.h"

namespace Espfc::Sensor {

class AccelSensor: public BaseSensor
{
  public:
    AccelSensor(Model& model);
    
    int begin();
    int update();
    int read();
    int filter();

  private:
    void calibrate();

    Model& _model;
    Device::GyroDevice * _gyro;
    Utils::Filter _filter[AXIS_COUNT_RPY];
};

}
