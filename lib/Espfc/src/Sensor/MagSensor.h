#pragma once

#include "Model.h"
#include "BaseSensor.h"
#include "Device/MagDevice.h"

namespace Espfc {

namespace Sensor {

class MagSensor: public BaseSensor
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
    Device::MagDevice * _mag;
};

}

}
