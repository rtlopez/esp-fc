#ifndef _ESPFC_SENSOR_GYRO_SENSOR_H_
#define _ESPFC_SENSOR_GYRO_SENSOR_H_

#include "BaseSensor.h"
#include "Device/GyroDevice.h"

#define ESPFC_FUZZY_ACCEL_ZERO 0.05
#define ESPFC_FUZZY_GYRO_ZERO  0.20

namespace Espfc {

namespace Sensor {

class GyroSensor: public BaseSensor
{
  public:
    GyroSensor(Model& model): _model(model) {}

    int begin()
    {
      _gyro = _model.state.gyroDev;
      if(!_gyro) return 0;

      _gyro->setDLPFMode(_model.config.gyroDlpf);
      _gyro->setRate(_gyro->getRate());

      switch(_model.config.gyroFsr)
      {
        case GYRO_FS_2000: _model.state.gyroScale = radians(2000.f) / 32768.f; break;
        case GYRO_FS_1000: _model.state.gyroScale = radians(1000.f) / 32768.f; break;
        case GYRO_FS_500:  _model.state.gyroScale =  radians(500.f) / 32768.f; break;
        case GYRO_FS_250:  _model.state.gyroScale =  radians(250.f) / 32768.f; break;
      }
      _gyro->setFullScaleGyroRange(_model.config.gyroFsr);

      _model.state.gyroCalibrationState = CALIBRATION_START; // calibrate gyro on start
      _model.state.gyroCalibrationRate = _model.state.loopTimer.rate;
      _model.state.gyroBiasAlpha = 5.0f / _model.state.gyroCalibrationRate;

      _model.logger.info().log(F("GYRO INIT")).log(FPSTR(Device::GyroDevice::getName(_gyro->getType()))).log(_model.config.gyroDlpf).log(_gyro->getRate()).log(_model.state.gyroTimer.rate).logln(_model.state.gyroTimer.interval);

      _idx = 0;
      _count = std::min(_model.config.loopSync, (int8_t)8);

      return 1;
    }

    int update()
    {
      int status = read();

      if (status) filter();

      return status;
    }

    int read()
    {
      if(!_model.gyroActive()) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_GYRO_READ);

      _gyro->readGyro(_model.state.gyroRaw);

      align(_model.state.gyroRaw, _model.config.gyroAlign);

      VectorFloat input = (VectorFloat)_model.state.gyroRaw * _model.state.gyroScale;

      // moving average filter
      if(_count > 1)
      {
        _sum -= _samples[_idx];
        _sum += input;
        _samples[_idx] = input;
        if (++_idx == _count) _idx = 0;

        _model.state.gyroSampled = _sum / (float)_count;
      }
      else
      {
        _model.state.gyroSampled = input;
      }

      return 1;
    }

    int filter()
    {
      if(!_model.gyroActive()) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_GYRO_FILTER);

      _model.state.gyro = _model.state.gyroSampled;

      calibrate();

      bool dynamicFilterEnabled = _model.isActive(FEATURE_DYNAMIC_FILTER);
      bool dynamicFilterDebug = _model.config.debugMode == DEBUG_FFT_FREQ;
      bool dynamicFilterUpdate = dynamicFilterEnabled && _model.state.dynamicFilterTimer.check();

      // filtering
      for(size_t i = 0; i < 3; ++i)
      {
        if(_model.config.debugMode == DEBUG_GYRO_RAW)
        {
          _model.state.debug[i] = _model.state.gyroRaw[i];
        }
        if(_model.config.debugMode == DEBUG_GYRO_SCALED)
        {
          _model.state.debug[i] = lrintf(degrees(_model.state.gyro[i]));
        }
        _model.state.gyro.set(i, _model.state.gyroFilter3[i].update(_model.state.gyro[i]));

        if(dynamicFilterEnabled || dynamicFilterDebug)
        {
          dynamicFilterAnalyze((Axis)i, dynamicFilterDebug);
          if(dynamicFilterUpdate) dynamicFilterApply((Axis)i);
          if(dynamicFilterEnabled) {
            _model.state.gyro.set(i, _model.state.gyroDynamicFilter[i].update(_model.state.gyro[i]));
            _model.state.gyro.set(i, _model.state.gyroDynamicFilter2[i].update(_model.state.gyro[i]));
          }
        }
        _model.state.gyro.set(i, _model.state.gyroNotch1Filter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroNotch2Filter[i].update(_model.state.gyro[i]));
        if(_model.config.debugMode == DEBUG_GYRO_FILTERED)
        {
          _model.state.debug[i] = lrintf(degrees(_model.state.gyro[i]));
        }
        _model.state.gyro.set(i, _model.state.gyroFilter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroFilter2[i].update(_model.state.gyro[i]));
        if(_model.accelActive())
        {
          _model.state.gyroImu.set(i, _model.state.gyroImuFilter[i].update(_model.state.gyro[i]));
        }
      }

      return 1;
    }

  private:
    void dynamicFilterAnalyze(Axis i, bool debug)
    {
      _model.state.gyroAnalyzer[i].update(_model.state.gyro[i]);
      if(debug)
      {
        if(i == 0)
        {
          _model.state.debug[0] = _model.state.gyroAnalyzer[0].freq;
          _model.state.debug[2] = lrintf(degrees(_model.state.gyroAnalyzer[0].noise));
          _model.state.debug[3] = lrintf(degrees(_model.state.gyro[0]));
        }
        else if(i == 1)
        {
          _model.state.debug[1] = _model.state.gyroAnalyzer[1].freq;
        }
      }
    }

    void dynamicFilterApply(Axis i)
    {
      const float freq = _model.state.gyroAnalyzer[i].freq;
      const float bw = 0.5f * (freq / (_model.config.dynamicFilter.q * 0.01)); // half bandwidth
      if(_model.config.dynamicFilter.width > 0 && _model.config.dynamicFilter.width < 30) {
        const float w = 0.005f * _model.config.dynamicFilter.width; // half witdh
        const float freq1 = freq * (1.0f - w);
        const float freq2 = freq * (1.0f + w);
        _model.state.gyroDynamicFilter[i].reconfigure(freq1, freq1 - bw);
        _model.state.gyroDynamicFilter2[i].reconfigure(freq2, freq2 - bw);
      } else {
        _model.state.gyroDynamicFilter[i].reconfigure(freq, freq - bw);
      }
    }

    void calibrate()
    {
      switch(_model.state.gyroCalibrationState)
      {
        case CALIBRATION_IDLE:
          _model.state.gyro -= _model.state.gyroBias;
          break;
        case CALIBRATION_START:
          //_model.state.gyroBias = VectorFloat();
          _model.state.gyroBiasSamples = 2 * _model.state.gyroCalibrationRate;
          _model.state.gyroCalibrationState = CALIBRATION_UPDATE;
          break;
        case CALIBRATION_UPDATE:
          {
            VectorFloat deltaAccel = _model.state.accel - _model.state.accelPrev;
            _model.state.accelPrev = _model.state.accel;
            if(deltaAccel.getMagnitude() < ESPFC_FUZZY_ACCEL_ZERO && _model.state.gyro.getMagnitude() < ESPFC_FUZZY_GYRO_ZERO)
            {
              _model.state.gyroBias += (_model.state.gyro - _model.state.gyroBias) * _model.state.gyroBiasAlpha;
              _model.state.gyroBiasSamples--;
            }
            if(_model.state.gyroBiasSamples <= 0) _model.state.gyroCalibrationState = CALIBRATION_APPLY;
          }
          break;
        case CALIBRATION_APPLY:
          _model.state.gyroCalibrationState = CALIBRATION_SAVE;
          break;
        case CALIBRATION_SAVE:
          _model.finishCalibration();
          _model.state.gyroCalibrationState = CALIBRATION_IDLE;
          break;
        default:
          _model.state.gyroCalibrationState = CALIBRATION_IDLE;
          break;
      }
    }

    size_t _idx;
    size_t _count;
    VectorFloat _sum;
    VectorFloat _samples[8];

    Model& _model;
    Device::GyroDevice * _gyro;
};

}

}
#endif