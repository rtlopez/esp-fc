#include "BaroSensor.h"
#include <functional>

namespace Espfc {

namespace Sensor {

BaroSensor::BaroSensor(Model& model): _model(model), _state(BARO_STATE_INIT), _counter(0) {}

int BaroSensor::begin()
{
  _model.state.baro.rate = 0;
  if(!_model.baroActive()) return 0;
  _baro = _model.state.baro.dev;
  if(!_baro) return 0;

  _baro->setMode(BARO_MODE_TEMP);
  int delay = _baro->getDelay(BARO_MODE_TEMP) + _baro->getDelay(BARO_MODE_PRESS);
  int toGyroRate = (delay / _model.state.gyro.timer.interval) + 1; // number of gyro readings per cycle
  int interval = _model.state.gyro.timer.interval * toGyroRate;
  _model.state.baro.rate = 1000000 / interval;

  _model.state.baro.altitudeBiasSamples = 2 * _model.state.baro.rate;

  // TODO: move filters to BaroState
  auto internalFilter = FILTER_PT1;
  auto internalCutoff = std::max(_model.state.baro.rate / 10, (int32_t)1);
  _temperatureFilter.begin(FilterConfig(internalFilter, internalCutoff), _model.state.baro.rate);
  _pressureFilter.begin(FilterConfig(internalFilter, internalCutoff), _model.state.baro.rate);

  _altitudeFilter.begin(_model.config.baro.filter, _model.state.baro.rate);

  _model.logger.info().log(F("BARO INIT")).log(FPSTR(Device::BaroDevice::getName(_baro->getType()))).log(_baro->getAddress()).log(toGyroRate).log(_model.state.baro.rate).logln(_model.config.baro.filter.freq);

  return 1;
}

int BaroSensor::update()
{
  int status = read();

  return status;
}

int BaroSensor::read()
{
  if(!_baro || !_model.baroActive()) return 0;

  if(_wait > micros()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BARO);

  if(_model.config.debug.mode == DEBUG_BARO)
  {
    _model.state.debug[3] = _state;
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

void BaroSensor::readTemperature()
{
  _model.state.baro.temperatureRaw = _baro->readTemperature();
  _model.state.baro.temperature = _temperatureFilter.update(_model.state.baro.temperatureRaw);
}

void BaroSensor::readPressure()
{
  _model.state.baro.pressureRaw = _baro->readPressure();
  _model.state.baro.pressure = _pressureFilter.update(_model.state.baro.pressureRaw);
}

void BaroSensor::updateAltitude()
{
  _model.state.baro.altitudeRaw = Utils::toAltitude(_model.state.baro.pressure);
  if(_model.state.baro.altitudeBiasSamples > 0)
  {
    _model.state.baro.altitudeBiasSamples--;
    _model.state.baro.altitudeBias += (_model.state.baro.altitudeRaw - _model.state.baro.altitudeBias) * (5.0f / _model.state.baro.rate);
  }
  _model.state.baro.altitude = _altitudeFilter.update(_model.state.baro.altitudeRaw - _model.state.baro.altitudeBias);

  if(_model.config.debug.mode == DEBUG_BARO)
  {
    _model.state.debug[0] = lrintf((_model.state.baro.altitudeRaw - _model.state.baro.altitudeBias) * 100.f); // cm
    _model.state.debug[1] = lrintf(_model.state.baro.pressureRaw * 0.1f); // hPa x 10
    _model.state.debug[2] = lrintf(_model.state.baro.temperatureRaw * 100.f); // deg C x 100
  }
}

}

}
