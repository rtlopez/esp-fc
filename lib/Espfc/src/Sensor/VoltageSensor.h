#ifndef _ESPFC_SENSOR_VOLTAGE_SENSOR_H_
#define _ESPFC_SENSOR_VOLTAGE_SENSOR_H_

#include "Model.h"
#include "BaseSensor.h"
#include <algorithm>

namespace Espfc {

namespace Sensor {

class VoltageSensor: public BaseSensor
{
  public:
    VoltageSensor(Model& model): _model(model) {}

    int begin()
    {
      _model.state.battery.timer.setRate(20);
      _model.state.battery.samples = 40;

      return 1;
    }

    int update()
    {
      return 0;

      if(!_model.state.battery.timer.check()) return 0;

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
      val = constrain(val, 0.f, 255.f);
      val = (val * alpha + _model.state.battery.voltage * (1.f - alpha)); // smooth
      _model.state.battery.voltage = (uint8_t)lrintf(val);

      // cell count detection
      if(_model.state.battery.samples > 0)
      {
        _model.state.battery.cells = ((int)_model.state.battery.voltage + 40) / 42;  // round
        _model.state.battery.samples--;
      }
      _model.state.battery.cellVoltage = _model.state.battery.voltage / constrain(_model.state.battery.cells, 1, 6);

      return 1;
    }

  private:
    Model& _model;
};

}

}

#endif