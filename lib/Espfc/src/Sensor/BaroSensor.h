#ifndef _ESPFC_SENSOR_BARO_SENSOR_H_
#define _ESPFC_SENSOR_BARO_SENSOR_H_

#include "BaseSensor.h"
#include "Device/BaroDevice.h"
#include "Filter.h"

namespace Espfc {

namespace Sensor {

class BaroSensor: public BaseSensor
{
  public:
    enum BaroState
    {
      BARO_STATE_INIT,
      BARO_STATE_TEMP_SET,
      BARO_STATE_TEMP_GET,
      BARO_STATE_PRESS_SET,
      BARO_STATE_PRESS_GET,
    };

    BaroSensor(Model& model): _model(model), _state(BARO_STATE_INIT), _counter(0) {}

    int begin()
    {
      _baro = Hardware::getBaroDevice(_model);
      if(!_baro) return 0;

      _baro->setMode(BARO_MODE_TEMP);
      int interval = _baro->getDelay() + _model.state.gyroTimer.interval;
      int rate = 1000000 / interval;

      FilterConfig tfc;
      tfc.type = FILTER_FIR2;
      _temperatureFilter.begin(tfc, rate);

      FilterConfig pfc;
      pfc.type = FILTER_BIQUAD;
      pfc.freq = 10;
      _pressureFilterLpf.begin(pfc, rate);

      FilterConfig mfc;
      mfc.type = FILTER_MEDIAN3;
      _pressureFilterMedian.begin(mfc, rate);

      return 1;
    }

    int update()
    {
      if(!_model.baroActive()) return 0;

      uint32_t now = micros();
      if(_wait > now) return 0;

      switch(_state)
      {
        case BARO_STATE_INIT:
          _state = BARO_STATE_TEMP_SET;
          break;
        case BARO_STATE_TEMP_SET:
          _baro->setMode(BARO_MODE_TEMP);
          _wait = micros() + _baro->getDelay();
          _state = BARO_STATE_TEMP_GET;
          break;
        case BARO_STATE_TEMP_GET:
          _model.state.baroTemperatureRaw = _baro->readTemperature();
          _model.state.baroTemperature = _temperatureFilter.update(_model.state.baroTemperatureRaw);
          updateAltitude();
          _state = BARO_STATE_PRESS_SET;
          _counter = 10;
          break;
        case BARO_STATE_PRESS_SET:
          _baro->setMode(BARO_MODE_PRESS);
          _wait = micros() + _baro->getDelay();
          _state = BARO_STATE_PRESS_GET;
          break;
        case BARO_STATE_PRESS_GET:
          _model.state.baroPressureRaw = _pressureFilterMedian.update(_baro->readPreassure());
          updateAltitude();
          _counter--;
          _state = _counter == 0 ? BARO_STATE_TEMP_SET : BARO_STATE_PRESS_SET;
          break;
        default:
          _state = BARO_STATE_INIT;
          break;
      }

      return 1;
    }

  private:
    void updateAltitude()
    {
      _model.state.baroPressure = _pressureFilterLpf.update(_model.state.baroPressureRaw);
      _model.state.baroAltitude = _baro->getAltitude(_model.state.baroPressure);
      if(_model.state.baroAlititudeBiasSamples--)
      {
        _model.state.baroAltitudeBias += (_model.state.baroAltitude - _model.state.baroAltitudeBias) * 0.2;
      }
      _model.state.baroAltitude -= _model.state.baroAltitudeBias;
    }

    Model& _model;
    Device::BaroDevice * _baro;
    BaroState _state;
    Filter _pressureFilterMedian;
    Filter _pressureFilterLpf;
    Filter _temperatureFilter;
    uint32_t _wait;
    int32_t _counter;
};

}

}
#endif