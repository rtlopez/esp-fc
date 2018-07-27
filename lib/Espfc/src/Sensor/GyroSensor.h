#ifndef _ESPFC_SENSOR_GYRO_SENSOR_H_
#define _ESPFC_SENSOR_GYRO_SENSOR_H_

#include "BaseSensor.h"
#include "Device/GyroDevice.h"

#include <math.h>

#define ESPFC_FUZZY_ACCEL_ZERO 0.05
#define ESPFC_FUZZY_GYRO_ZERO 0.20

namespace Espfc {

namespace Sensor {

class GyroSensor: public BaseSensor
{
  public:
    GyroSensor(Model& model): _model(model) {}

    int begin()
    {
      _gyro = Hardware::getGyroDevice(_model);
      if(!_gyro) return 0;

      _gyro->setDLPFMode(_model.config.gyroDlpf);
      _gyro->setRate(_model.state.gyroDivider - 1);

      switch(_model.config.gyroFsr)
      {
        case GYRO_FS_2000: _model.state.gyroScale = M_PI /  (16.384 * 180.0); break;
        case GYRO_FS_1000: _model.state.gyroScale = M_PI /  (32.768 * 180.0); break;
        case GYRO_FS_500:  _model.state.gyroScale = M_PI /  (65.535 * 180.0); break;
        case GYRO_FS_250:  _model.state.gyroScale = M_PI / (131.072 * 180.0); break;
      }
      _gyro->setFullScaleGyroRange(_model.config.gyroFsr);

      _model.logger.info().log(F("GYRO RATE")).log(_model.config.gyroDlpf).log(_model.state.gyroDivider).log(_model.state.gyroTimer.rate).logln(_model.state.gyroTimer.interval);
      _model.logger.info().log(F("GYRO INIT")).log(_model.config.gyroDev).logln(_model.state.gyroPresent);

      return 1;
    }

    int update()
    {
      if(!_model.gyroActive()) return 0;

      {
        Stats::Measure measure(_model.state.stats, COUNTER_GYRO_READ);
        _gyro->readGyro(_model.state.gyroRaw);
      }

      {
        Stats::Measure measure(_model.state.stats, COUNTER_GYRO_FILTER);

        if(!_model.gyroActive()) return 0;

        align(_model.state.gyroRaw, _model.config.gyroAlign);

        _model.state.gyro = (VectorFloat)_model.state.gyroRaw  * _model.state.gyroScale;
        if(_model.state.gyroBiasSamples > 0) // calibration
        {
          VectorFloat deltaAccel = _model.state.accel - _model.state.accelPrev;
          _model.state.accelPrev = _model.state.accel;
          if(deltaAccel.getMagnitude() < ESPFC_FUZZY_ACCEL_ZERO && _model.state.gyro.getMagnitude() < ESPFC_FUZZY_GYRO_ZERO)
          {
            _model.state.gyroBias += (_model.state.gyro - _model.state.gyroBias) * _model.state.gyroBiasAlpha;
            _model.state.gyroBiasSamples--;
            if(_model.state.gyroBiasSamples == 0)
            {
              _model.state.buzzer.push(BEEPER_GYRO_CALIBRATED);
              _model.logger.info().log(F("GYRO CAL")).log(_model.state.gyroBias.x).log(_model.state.gyroBias.y).logln(_model.state.gyroBias.z);
              //_fusion.restoreGain();
            }
          }
        }
        _model.state.gyro -= _model.state.gyroBias;

        // filtering
        for(size_t i = 0; i < 3; ++i)
        {
          if(_model.config.debugMode == DEBUG_GYRO)
          {
            _model.state.debug[i] = _model.state.gyroRaw[i];
          }
          if(_model.config.debugMode == DEBUG_NOTCH)
          {
            _model.state.debug[i] = lrintf(degrees(_model.state.gyro[i]));
          }
          _model.state.gyro.set(i, _model.state.gyroNotch1Filter[i].update(_model.state.gyro[i]));
          _model.state.gyro.set(i, _model.state.gyroNotch2Filter[i].update(_model.state.gyro[i]));
          _model.state.gyro.set(i, _model.state.gyroFilter[i].update(_model.state.gyro[i]));
          _model.state.gyro.set(i, _model.state.gyroFilter2[i].update(_model.state.gyro[i]));
        }
      }     

      return 1;
    }

  private:
    Model& _model;
    Device::GyroDevice * _gyro;
};

}

}
#endif