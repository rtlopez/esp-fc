#ifndef _ESPFC_SENSOR_ACCEL_SENSOR_H_
#define _ESPFC_SENSOR_ACCEL_SENSOR_H_

#include "BaseSensor.h"
#include "Device/GyroDevice.h"

namespace Espfc {

namespace Sensor {

class AccelSensor: public BaseSensor
{
  public:
    AccelSensor(Model& model): _model(model) {}
    
    int begin()
    {
      _model.state.accel.z = ACCEL_G;

      _gyro = Hardware::getGyroDevice(_model);
      if(!_gyro) return 0;

      switch(_model.config.accelFsr)
      {
        case ACCEL_FS_16: _model.state.accelScale = 16.f * ACCEL_G / 32768.f; break;
        case ACCEL_FS_8:  _model.state.accelScale =  8.f * ACCEL_G / 32768.f; break;
        case ACCEL_FS_4:  _model.state.accelScale =  4.f * ACCEL_G / 32768.f; break;
        case ACCEL_FS_2:  _model.state.accelScale =  2.f * ACCEL_G / 32768.f; break;
      }
      _gyro->setFullScaleAccelRange(_model.config.accelFsr);

      for(size_t i = 0; i < 3; i++)
      {
        _filter[i].begin(FilterConfig(FILTER_FIR2, 1), _model.state.accelTimer.rate);
      }

      _model.state.accelBiasAlpha = _model.state.accelTimer.rate > 0 ? 5.0f / _model.state.accelTimer.rate : 0;
      _model.state.accelCalibrationState = CALIBRATION_IDLE;

      _model.logger.info().log(F("ACCEL INIT")).log(FPSTR(Device::GyroDevice::getName(_gyro->getType()))).log(_model.state.accelTimer.rate).log(_model.state.accelTimer.interval).logln(_model.state.accelPresent);

      return 1;
    }

    int update()
    {
      if(!_model.accelActive()) return 0;

      if(!_model.state.accelTimer.check()) return 0;

      {
        Stats::Measure measure(_model.state.stats, COUNTER_ACCEL_READ);
        _gyro->readAccel(_model.state.accelRaw);
      }

      {
        Stats::Measure measure(_model.state.stats, COUNTER_ACCEL_FILTER);

        align(_model.state.accelRaw, _model.config.accelAlign);

        _model.state.accel = (VectorFloat)_model.state.accelRaw * _model.state.accelScale;
        for(size_t i = 0; i < 3; i++)
        {
          if(_model.config.debugMode == DEBUG_ACCELEROMETER)
          {
            _model.state.debug[i] = _model.state.accelRaw[i];
          }
          _model.state.accel.set(i, _filter[i].update(_model.state.accel[i]));
          _model.state.accel.set(i, _model.state.accelFilter[i].update(_model.state.accel[i]));
        }

        calibrate();
      }

      return 1;
    }

  private:
    void calibrate()
    {
      switch(_model.state.accelCalibrationState)
      {
        case CALIBRATION_IDLE:
          _model.state.accel -= _model.state.accelBias;
          break;
        case CALIBRATION_START:
          _model.state.accelBias = VectorFloat(0, 0, ACCEL_G);
          _model.state.accelBiasSamples = 2 * _model.state.accelTimer.rate;
          _model.state.accelCalibrationState = CALIBRATION_UPDATE;
          break;
        case CALIBRATION_UPDATE:
          _model.state.accelBias += (_model.state.accel - _model.state.accelBias) * _model.state.accelBiasAlpha;
          _model.state.accelBiasSamples--;
          if(_model.state.accelBiasSamples <= 0) _model.state.accelCalibrationState = CALIBRATION_APPLY;
          break;
        case CALIBRATION_APPLY:
          _model.state.accelBias.z -= ACCEL_G;
          _model.state.accelCalibrationState = CALIBRATION_SAVE;
          break;
        case CALIBRATION_SAVE:
          _model.finishCalibration();
          _model.state.accelCalibrationState = CALIBRATION_IDLE;
          break;
        default:
          _model.state.accelCalibrationState = CALIBRATION_IDLE;
          break;
      }
    }

    Model& _model;
    Device::GyroDevice * _gyro;
    Filter _filter[3];
};

}

}
#endif