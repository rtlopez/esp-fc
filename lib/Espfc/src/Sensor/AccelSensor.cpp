#include "Sensor/AccelSensor.h"
#include "Utils/FilterHelper.h"

namespace Espfc::Sensor {

static constexpr float ESPFC_FUZZY_ACCEL_ZERO = 0.05f;
static constexpr float ESPFC_FUZZY_GYRO_ZERO = 0.20f;

AccelSensor::AccelSensor(Model& model): _model(model) {}

int AccelSensor::begin()
{
  _model.state.accel.adc.z = ACCEL_G;

  _gyro = _model.state.gyro.dev;
  if(!_gyro) return 0;

  _model.state.accel.scale = 16.f * ACCEL_G / 32768.f;

  for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
    _filter[i].begin(FilterConfig(FILTER_FIR2, 1), _model.state.accel.timer.rate);
    _model.state.accel.filter[i].begin(_model.config.accel.filter, _model.state.accel.timer.rate);
  }

  _model.state.accel.biasAlpha = 5.0f / _model.state.accel.timer.rate;
  _model.state.accel.calibrationState = CALIBRATION_IDLE;

  _model.logger.info().log(F("ACCEL INIT")).log(FPSTR(Device::GyroDevice::getName(_gyro->getType()))).log(_gyro->getAddress()).log(_model.state.accel.timer.rate).log(_model.state.accel.timer.interval).logln(_model.state.accel.present);

  return 1;
}

int FAST_CODE_ATTR AccelSensor::update()
{
  int status = read();

  if (status) filter();

  return status;
}

int FAST_CODE_ATTR AccelSensor::read()
{
  if(!_model.accelActive()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_ACCEL_READ);
  _gyro->readAccel(_model.state.accel.raw);

  return 1;
}

int FAST_CODE_ATTR AccelSensor::filter()
{
  if(!_model.accelActive()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_ACCEL_FILTER);

  _model.state.accel.adc = (VectorFloat)_model.state.accel.raw * _model.state.accel.scale;

  align(_model.state.accel.adc, _model.config.gyro.align);
  _model.state.accel.adc = _model.state.boardAlignment.apply(_model.state.accel.adc);

  for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
    if(_model.config.debug.mode == DEBUG_ACCELEROMETER)
    {
      _model.state.debug[i] = _model.state.accel.raw[i];
    }
    _model.state.accel.adc.set(i, _filter[i].update(_model.state.accel.adc[i]));
    _model.state.accel.adc.set(i, _model.state.accel.filter[i].update(_model.state.accel.adc[i]));
  }

  calibrate();

  return 1;
}

void FAST_CODE_ATTR AccelSensor::calibrate()
{
  switch(_model.state.accel.calibrationState)
  {
    case CALIBRATION_IDLE:
      _model.state.accel.adc -= _model.state.accel.bias;
      break;
    case CALIBRATION_START:
      _model.state.accel.bias = VectorFloat(0, 0, ACCEL_G);
      _model.state.accel.biasSamples = 2 * _model.state.accel.timer.rate;
      _model.state.accel.calibrationState = CALIBRATION_UPDATE;
      break;
    case CALIBRATION_UPDATE:
      _model.state.accel.bias += (_model.state.accel.adc - _model.state.accel.bias) * _model.state.accel.biasAlpha;
      _model.state.accel.biasSamples--;
      if(_model.state.accel.biasSamples <= 0) _model.state.accel.calibrationState = CALIBRATION_APPLY;
      break;
    case CALIBRATION_APPLY:
      _model.state.accel.bias.z -= ACCEL_G;
      _model.state.accel.calibrationState = CALIBRATION_SAVE;
      break;
    case CALIBRATION_SAVE:
      _model.finishCalibration();
      _model.state.accel.calibrationState = CALIBRATION_IDLE;
      break;
    default:
      _model.state.accel.calibrationState = CALIBRATION_IDLE;
      break;
  }
}

}
