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
      _model.state.battery.timer.setRate(50);
      _model.state.battery.samples = 50;

      if(_model.config.vbatSource != 1 || _model.config.pin[PIN_INPUT_ADC_0] == -1)
      {
        return 0;
      }

      return 1;
    }

    int update()
    {
      if(_model.config.vbatSource != 1 || _model.config.pin[PIN_INPUT_ADC_0] == -1)
      {
        return 0;
      }

      if(!_model.state.battery.timer.check()) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_BATTERY);

      // wemos d1 mini has divider 3.2:1 (220k:100k)
      // additionaly I've used divider 5.7:1 (4k7:1k)
      // total should equals ~18.24:1, 73:4 resDiv:resMult should be ideal,
      // but ~52:1 is real, did I miss something?
      const float alpha = 0.33f;
      _model.state.battery.rawVoltage = analogRead(_model.config.pin[PIN_INPUT_ADC_0]);
      float realVal = _model.state.battery.rawVoltage * ESPFC_ADC_SCALE;

      float val = realVal * _model.config.vbatScale * 0.1f;
      val *= _model.config.vbatResMult;
      val /= _model.config.vbatResDiv;
      _model.state.battery.voltage += (val - _model.state.battery.voltage) * alpha;

      // cell count detection
      if(_model.state.battery.samples > 0)
      {
        _model.state.battery.cells = std::ceil(_model.state.battery.voltage / 4.2f);
        _model.state.battery.samples--;
      }
      _model.state.battery.cellVoltage = lrintf((_model.state.battery.voltage * 10.f) / constrain(_model.state.battery.cells, 1, 6));

      return 1;
    }

  private:
    Model& _model;
};

}

}

#endif