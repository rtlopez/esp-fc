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
      _model.state.baroRate = 0;
      if(!_model.baroActive()) return 0;
      _baro = _model.state.baroDev;
      if(!_baro) return 0;

      _baro->setMode(BARO_MODE_TEMP);
      int delay = _baro->getDelay(BARO_MODE_TEMP) + _baro->getDelay(BARO_MODE_PRESS);
      int toGyroRate = (delay / _model.state.gyroTimer.interval) + 1; // number of gyro readings per cycle
      int interval = _model.state.gyroTimer.interval * toGyroRate;
      _model.state.baroRate = 1000000 / interval;

      _model.state.baroAltitudeBiasSamples = 2 * _model.state.baroRate;

      _temperatureFilter.begin(FilterConfig(FILTER_PT1, _model.state.baroRate * 0.1f), _model.state.baroRate);
      _pressureFilter.begin(FilterConfig(FILTER_PT1, _model.state.baroRate * 0.1f), _model.state.baroRate);
      _altitudeFilter.begin(_model.config.baroFilter, _model.state.baroRate);

      _model.logger.info().log(F("BARO INIT")).log(FPSTR(Device::BaroDevice::getName(_baro->getType()))).log(_baro->getAddress()).log(toGyroRate).log(_model.state.baroRate).logln(_model.config.baroFilter.freq);

      return 1;
    }

    int update()
    {
      int status = read();

      return status;
    }

    int read()
    {
      if(!_baro || !_model.baroActive()) return 0;
      
      if(_wait > micros()) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_BARO);

      if(_model.config.debugMode == DEBUG_BARO)
      {
        _model.state.debug[0] = _state;
      }

      switch(_state)
      {
        case BARO_STATE_INIT:
          _baro->setMode(BARO_MODE_TEMP);
          _state = BARO_STATE_TEMP_GET;
          _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
          return 0;
        case BARO_STATE_TEMP_GET:
          readTemperature();
          _baro->setMode(BARO_MODE_PRESS);
          _state = BARO_STATE_PRESS_GET;
          _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
          _counter = 1;
          return 1;
        case BARO_STATE_PRESS_GET:
          readPressure();
          updateAltitude();
          if(--_counter > 0)
          {
            _baro->setMode(BARO_MODE_PRESS);
            _state = BARO_STATE_PRESS_GET;
            _wait = micros() + _baro->getDelay(BARO_MODE_PRESS);
          }
          else
          {
            _baro->setMode(BARO_MODE_TEMP);
            _state = BARO_STATE_TEMP_GET;
            _wait = micros() + _baro->getDelay(BARO_MODE_TEMP);
          }
          return 1;
          break;
        default:
          _state = BARO_STATE_INIT;
          break;
      }

      return 0;
    }

  private:
    void readTemperature()
    {
      _model.state.baroTemperatureRaw = _baro->readTemperature();
      _model.state.baroTemperature = _temperatureFilter.update(_model.state.baroTemperatureRaw);
    }

    void readPressure()
    {
      _model.state.baroPressureRaw = _baro->readPressure();
      _model.state.baroPressure = _pressureFilter.update(_model.state.baroPressureRaw);
    }

    void updateAltitude()
    {
      _model.state.baroAltitudeRaw = _altitudeFilter.update(Math::toAltitude(_model.state.baroPressure));
      if(_model.state.baroAltitudeBiasSamples > 0)
      {
        _model.state.baroAltitudeBiasSamples--;
        _model.state.baroAltitudeBias += (_model.state.baroAltitudeRaw - _model.state.baroAltitudeBias) * 0.2f;
      }
      _model.state.baroAltitude = _model.state.baroAltitudeRaw - _model.state.baroAltitudeBias;

      if(_model.config.debugMode == DEBUG_BARO)
      {
        _model.state.debug[1] = lrintf(_model.state.baroPressureRaw * 0.1f); // hPa x 10
        _model.state.debug[2] = lrintf(_model.state.baroTemperatureRaw * 100.f); // deg C x 100
        _model.state.debug[3] = lrintf(_model.state.baroAltitudeRaw * 10.f);
      }
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