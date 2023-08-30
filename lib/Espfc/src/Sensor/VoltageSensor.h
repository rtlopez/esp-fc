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
    enum State {
      VBAT,
      IBAT
    };
    VoltageSensor(Model& model): _model(model) {}

    int begin()
    {
      _model.state.battery.timer.setRate(100);
      _model.state.battery.samples = 50;

      _vFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _vFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);

      _iFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _iFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);

      _state = VBAT;

      return 1;
    }

    int update()
    {
      if(!_model.state.battery.timer.check()) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_BATTERY);

      switch(_state) {
        case VBAT:
          _state = IBAT;
          return readVbat();
        case IBAT:
          _state = VBAT;
          return readIbat();
      }

      return 0;
    }

    int readVbat()
    {
#ifdef ESPFC_ADC_0
      if(_model.config.vbatSource != 1 || _model.config.pin[PIN_INPUT_ADC_0] == -1) return 0;
      // wemos d1 mini has divider 3.2:1 (220k:100k)
      // additionaly I've used divider 5.7:1 (4k7:1k)
      // total should equals ~18.24:1, 73:4 resDiv:resMult should be ideal,
      // but ~52:1 is real, did I miss something?
      _model.state.battery.rawVoltage = analogRead(_model.config.pin[PIN_INPUT_ADC_0]);
      float volts = _vFilterFast.update(_model.state.battery.rawVoltage * ESPFC_ADC_SCALE);

      volts *= _model.config.vbatScale * 0.1f;
      volts *= _model.config.vbatResMult;
      volts /= _model.config.vbatResDiv;

      _model.state.battery.voltageUnfiltered = volts;
      _model.state.battery.voltage = _vFilter.update(_model.state.battery.voltageUnfiltered);

      // cell count detection
      if(_model.state.battery.samples > 0)
      {
        _model.state.battery.cells = std::ceil(_model.state.battery.voltage / 4.2f);
        _model.state.battery.samples--;
      }
      _model.state.battery.cellVoltage = _model.state.battery.voltage / constrain(_model.state.battery.cells, 1, 6);

      if(_model.config.debugMode == DEBUG_BATTERY)
      {
        _model.state.debug[0] = constrain(lrintf(_model.state.battery.voltageUnfiltered * 100.0f), 0, 32000);
        _model.state.debug[1] = constrain(lrintf(_model.state.battery.voltage * 100.0f), 0, 32000);
      }
      return 1;
#else
      return 0;
#endif
    }

    int readIbat()
    {
#ifdef ESPFC_ADC_1
      if(_model.config.ibatSource != 1 && _model.config.pin[PIN_INPUT_ADC_1] == -1) return 0;

      _model.state.battery.rawCurrent = analogRead(_model.config.pin[PIN_INPUT_ADC_1]);
      float volts = _iFilterFast.update(_model.state.battery.rawCurrent * ESPFC_ADC_SCALE);
      float milivolts = volts * 1000.0f;

      volts += _model.config.ibatOffset * 0.001f;
      volts *= _model.config.ibatScale * 0.1f;

      _model.state.battery.currentUnfiltered = volts;
      _model.state.battery.current = _iFilter.update(_model.state.battery.currentUnfiltered);

      if(_model.config.debugMode == DEBUG_CURRENT_SENSOR)
      {
        _model.state.debug[0] = lrintf(milivolts);
        _model.state.debug[1] = constrain(lrintf(_model.state.battery.currentUnfiltered * 100.0f), 0, 32000);
        _model.state.debug[2] = _model.state.battery.rawCurrent;
      }

      return 1;
#else
      return 0;
#endif
    }

  private:
    Model& _model;
    Filter _vFilterFast;
    Filter _vFilter;
    Filter _iFilterFast;
    Filter _iFilter;
    State _state;
};

}

}

#endif