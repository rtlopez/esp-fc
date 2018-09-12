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
      BARO_STATE_TEMP_GET,
      BARO_STATE_PRESS_GET,
    };

    BaroSensor(Model& model): _model(model), _state(BARO_STATE_INIT), _counter(0) {}

    int begin()
    {
      if(!_model.baroActive()) return 0;
      _baro = Hardware::getBaroDevice(_model);
      if(!_baro) return 0;

      _baro->setMode(BARO_MODE_TEMP);
      int delay = _baro->getDelay();
      int toGyroRate = (delay / _model.state.gyroTimer.interval) + 1; // number of gyro readings per cycle
      int interval = _model.state.gyroTimer.interval * toGyroRate;
      int rate = 1000000 / interval;

      _temperatureFilter.begin(FilterConfig(FILTER_PT1, 10), rate);
      _pressureFilter.begin(FilterConfig(FILTER_MEDIAN3, 10), rate);
      _altitudeFilter.begin(_model.config.baroFilter, rate);

      _model.logger.info().log(F("BARO INIT")).log(FPSTR(Device::BaroDevice::getName(_baro->getType()))).log(toGyroRate).log(rate).logln(_model.config.baroFilter.freq);

      return 1;
    }

    int update()
    {
      if(!_model.baroActive()) return 0;
      if(!_baro) return 0;
      
      Stats::Measure measure(_model.state.stats, COUNTER_BARO);
      
      if(_wait > micros()) return 0;

      switch(_state)
      {
        case BARO_STATE_INIT:
          _baro->setMode(BARO_MODE_TEMP);
          _state = BARO_STATE_TEMP_GET;
          _wait = micros() + _baro->getDelay();
          break;
        case BARO_STATE_TEMP_GET:
          readTemperature();
          updateTemperature();
          updateAltitude();
          _baro->setMode(BARO_MODE_PRESS);
          _state = BARO_STATE_PRESS_GET;
          _wait = micros() + _baro->getDelay();
          _counter = 9;
          break;
        case BARO_STATE_PRESS_GET:
          readPressure();
          updateAltitude();
          if(--_counter > 0)
          {
            _baro->setMode(BARO_MODE_PRESS);
            _state = BARO_STATE_PRESS_GET;
          }
          else
          {
            _baro->setMode(BARO_MODE_TEMP);
            _state = BARO_STATE_TEMP_GET;
          }
          _wait = micros() + _baro->getDelay();
          break;
        default:
          _state = BARO_STATE_INIT;
          break;
      }

      return 1;
    }

  private:
    void readTemperature()
    {
      _model.state.baroTemperatureRaw = _baro->readTemperature();
    }

    void updateTemperature()
    {
      _model.state.baroTemperature = _temperatureFilter.update(_model.state.baroTemperatureRaw);
    }

    void readPressure()
    {
      _model.state.baroPressureRaw = _baro->readPressure();
    }

    void updateAltitude()
    {
      _model.state.baroPressure = _pressureFilter.update(_model.state.baroPressureRaw);
      _model.state.baroAltitude = _altitudeFilter.update(_baro->getAltitude(_model.state.baroPressure));
      if(_model.state.baroAlititudeBiasSamples > 0)
      {
        _model.state.baroAlititudeBiasSamples--;
        _model.state.baroAltitudeBias += (_model.state.baroAltitude - _model.state.baroAltitudeBias) * 0.2f;
      }
      _model.state.baroAltitude -= _model.state.baroAltitudeBias;
    }

    Model& _model;
    Device::BaroDevice * _baro;
    BaroState _state;
    Filter _temperatureFilter;
    Filter _pressureFilter;
    Filter _altitudeFilter;
    uint32_t _wait;
    int32_t _counter;
};

}

}
#endif