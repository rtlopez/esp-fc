#include "MagSensor.h"
#include <cmath>

#ifdef abs
#undef abs
#endif

namespace Espfc {

namespace Sensor {

MagSensor::MagSensor(Model& model): _model(model) {}

int MagSensor::begin()
{
  if(!_model.magActive()) return 0;

  _mag = _model.state.mag.dev;
  if(!_mag) return 0;

  if(_model.state.mag.timer.rate < 5) return 0;

  // by default use eeprom calibration settings
  _model.state.mag.calibrationState = CALIBRATION_IDLE;
  _model.state.mag.calibrationValid = true;

  _model.logger.info().log(F("MAG INIT")).log(FPSTR(Device::MagDevice::getName(_mag->getType()))).log(_mag->getAddress()).logln(_model.state.mag.timer.rate);

  return 1;
}

int MagSensor::update()
{
  int status = read();

  if (status) filter();

  return status;
}

int MagSensor::read()
{
  if(!_mag || !_model.magActive() || !_model.state.mag.timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_MAG_READ);
  _mag->readMag(_model.state.mag.raw);

  return 1;
}

int MagSensor::filter()
{
  if(!_mag || !_model.magActive()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_MAG_FILTER);

  _model.state.mag.adc = _mag->convert(_model.state.mag.raw);

  align(_model.state.mag.adc, _model.config.mag.align);
  _model.state.mag.adc = _model.state.boardAlignment.apply(_model.state.mag.adc);

  for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
    _model.state.mag.adc.set(i, _model.state.mag.filter[i].update(_model.state.mag.adc[i]));
  }

  calibrate();

  return 1;
}

void MagSensor::calibrate()
{
  switch(_model.state.mag.calibrationState)
  {
    case CALIBRATION_IDLE:
      if(_model.state.mag.calibrationValid)
      {
        _model.state.mag.adc -= _model.state.mag.calibrationOffset;
        _model.state.mag.adc *= _model.state.mag.calibrationScale;
      }
      break;
    case CALIBRATION_START:
      resetCalibration();
      _model.state.mag.calibrationSamples = 30 * _model.state.mag.timer.rate;
      _model.state.mag.calibrationState = CALIBRATION_UPDATE;
      break;
    case CALIBRATION_UPDATE:
      updateCalibration();
      _model.state.mag.calibrationSamples--;
      if(_model.state.mag.calibrationSamples <= 0) _model.state.mag.calibrationState = CALIBRATION_APPLY;
      break;
    case CALIBRATION_APPLY:
      applyCalibration();
      _model.state.mag.calibrationState = CALIBRATION_SAVE;
      break;
    case CALIBRATION_SAVE:
      _model.finishCalibration();
      _model.state.mag.calibrationState = CALIBRATION_IDLE;
      break;
    default:
      _model.state.mag.calibrationState = CALIBRATION_IDLE;
      break;
  }
}

void MagSensor::resetCalibration()
{
  _model.state.mag.calibrationMin = VectorFloat();
  _model.state.mag.calibrationMax = VectorFloat();
  _model.state.mag.calibrationValid = false;
}

void MagSensor::updateCalibration()
{
  for(int i = 0; i < AXIS_COUNT_RPY; i++)
  {
    if(_model.state.mag.adc[i] < _model.state.mag.calibrationMin[i]) _model.state.mag.calibrationMin.set(i, _model.state.mag.adc[i]);
    if(_model.state.mag.adc[i] > _model.state.mag.calibrationMax[i]) _model.state.mag.calibrationMax.set(i, _model.state.mag.adc[i]);
  }
}

void MagSensor::applyCalibration()
{
  const float EPSILON = 0.001f;

  // verify calibration data and find biggest range
  float maxRange = -1;
  for(int i = 0; i < AXIS_COUNT_RPY; i++)
  {
    if(_model.state.mag.calibrationMin[i] > -EPSILON) return;
    if(_model.state.mag.calibrationMax[i] <  EPSILON) return;
    if((_model.state.mag.calibrationMax[i] - _model.state.mag.calibrationMin[i]) > maxRange)
    {
      maxRange = _model.state.mag.calibrationMax[i] - _model.state.mag.calibrationMin[i];
    }
  }

  // probably incomplete data, must be positive
  if(maxRange <= 0) return;

  VectorFloat scale(1.f, 1.f, 1.f);
  VectorFloat offset(0.f, 0.f, 0.f);

  for (int i = 0; i < AXIS_COUNT_RPY; i++)
  {
    const float range = (_model.state.mag.calibrationMax[i] - _model.state.mag.calibrationMin[i]);
    const float bias  = (_model.state.mag.calibrationMax[i] + _model.state.mag.calibrationMin[i]) * 0.5f;

    if(std::abs(range) < EPSILON) return;    // incomplete data

    scale.set(i, maxRange / range);     // makes everything the same range
    offset.set(i, bias);                // level bias
  }

  _model.state.mag.calibrationScale = scale;
  _model.state.mag.calibrationOffset = offset;
  _model.state.mag.calibrationValid = true;
}

}

}
