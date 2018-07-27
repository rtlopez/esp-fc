#ifndef _ESPFC_SENSOR_H_
#define _ESPFC_SENSOR_H_

#include <Arduino.h>
#include "Model.h"
#include "Filter.h"
#include "Fusion.h"
#include "Hardware.h"
#include "Sensor/GyroSensor.h"
#include "Sensor/AccelSensor.h"
#include "Sensor/MagSensor.h"

namespace Espfc {

class SensorManager
{
  public:
    SensorManager(Model& model): _model(model), _gyro(model), _accel(model), _mag(model), _fusion(model) {}

    int begin()
    {
      _gyro.begin();
      _accel.begin();
      _mag.begin();
      _fusion.begin();

      _model.state.accelRaw.z = 1.f;
      _model.state.battery.timer.setRate(20);
      _model.state.battery.samples = 40;
      
      return 1;
    }

    int update()
    {
      int ret = _gyro.update();

      if(!ret) return 0;

      _fusion.update();

      return 1;
    }

    int updateDelayed()
    {
      int accelUpdated = _accel.update();
      int magUpdated = _mag.update();

      if(magUpdated || accelUpdated)
      {
        _fusion.updateDelayed();
      }

      if(_model.state.battery.timer.check())
      {
        updateBattery();
      }

      _model.finishCalibration();
      return 1;
    }

  private: 
    void updateBattery()
    {
      // wemos d1 mini has divider 3.2:1 (220k:100k)
      // additionaly I've used divider 5.7:1 (4k7:1k)
      // total should equals ~18.24:1, 73:4 resDiv:resMult should be ideal,
      // but ~52:1 is real, did I miss something?
      const float alpha = 0.33f;
      float val = _model.state.battery.rawVoltage = analogRead(A0);
      val *= (int)_model.config.vbatScale;
      val /= 10.f;
      val *= _model.config.vbatResMult;
      val /= _model.config.vbatResDiv;
      val = Math::bound(val, 0.f, 255.f);
      val = (val * alpha + _model.state.battery.voltage * (1.f - alpha)); // smooth
      _model.state.battery.voltage = (uint8_t)lrintf(val);

      // cell count detection
      if(_model.state.battery.samples > 0)
      {
        _model.state.battery.cells = ((int)_model.state.battery.voltage + 40) / 42;  // round
        _model.state.battery.samples--;
      }
      _model.state.battery.cellVoltage = _model.state.battery.voltage / std::max((int)_model.state.battery.cells, 1);
    }

    Model& _model;
    Sensor::GyroSensor _gyro;
    Sensor::AccelSensor _accel;
    Sensor::MagSensor _mag;
    Fusion _fusion;
};

}

#endif
