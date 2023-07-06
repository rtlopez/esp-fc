#ifndef _ESPFC_SENSOR_GYRO_SENSOR_H_
#define _ESPFC_SENSOR_GYRO_SENSOR_H_

#include "BaseSensor.h"
#include "Device/GyroDevice.h"
#include "Math/Sma.h"
#include "Math/FreqAnalyzer.h"
#ifdef ESPFC_DSP
#include "Math/FFTAnalyzer.h"
#endif

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

      _sma.begin(_model.config.loopSync);

#ifdef ESPFC_DSP
      for(size_t i = 0; i < 3; i++)
      {
        _fft[i].begin(_model.state.loopTimer.rate, _model.config.dynamicFilter);
      }
#endif

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
      _model.state.gyroSampled = _sma.update(input);

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

      const int debugAxis = 1;

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
        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == debugAxis)
        {
          _model.state.debug[0] = lrintf(degrees(_model.state.gyro[i]));
        }

        _model.state.gyro.set(i, _model.state.gyroFilter3[i].update(_model.state.gyro[i]));
        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == debugAxis)
        {
          _model.state.debug[1] = lrintf(degrees(_model.state.gyro[i]));
        }

        _model.state.gyro.set(i, _model.state.gyroNotch1Filter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroNotch2Filter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroFilter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroFilter2[i].update(_model.state.gyro[i]));

        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == debugAxis)
        {
          _model.state.debug[2] = lrintf(degrees(_model.state.gyro[i]));
        }

        if(dynamicFilterEnabled || dynamicFilterDebug)
        {
#ifdef ESPFC_DSP
          int status = _fft[i].update(_model.state.gyro[i]);
          dynamicFilterUpdate = dynamicFilterEnabled && status;
          const float freq = _fft[i].freq;
#else
          _model.state.gyroAnalyzer[i].update(_model.state.gyro[i]);
          const float freq = _model.state.gyroAnalyzer[i].freq;
#endif
          if (dynamicFilterDebug)
          {
            _model.state.debug[i] = lrintf(freq);
            if (i == debugAxis) _model.state.debug[3] = lrintf(degrees(_model.state.gyro[i]));
          }
          if(dynamicFilterUpdate) dynamicFilterApply((Axis)i, freq);
          if(dynamicFilterEnabled)
          {
            _model.state.gyro.set(i, _model.state.gyroDynamicFilter[i].update(_model.state.gyro[i]));
            _model.state.gyro.set(i, _model.state.gyroDynamicFilter2[i].update(_model.state.gyro[i]));
          }
        }

        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == debugAxis)
        {
          _model.state.debug[3] = lrintf(degrees(_model.state.gyro[i]));
        }

        if(_model.config.debugMode == DEBUG_GYRO_FILTERED)
        {
          _model.state.debug[i] = lrintf(degrees(_model.state.gyro[i]));
        }
        if(_model.accelActive())
        {
          _model.state.gyroImu.set(i, _model.state.gyroImuFilter[i].update(_model.state.gyro[i]));
        }
      }

      return 1;
    }

  private:
    void dynamicFilterApply(Axis i, const float freq)
    {
      const float q = _model.config.dynamicFilter.q * 0.01;
      //const float bw = 0.5f * (freq / q)); // half bandwidth
      if(_model.config.dynamicFilter.width > 0 && _model.config.dynamicFilter.width < 30) {
        const float w = 0.005f * _model.config.dynamicFilter.width; // half witdh
        const float freq1 = freq * (1.0f - w);
        const float freq2 = freq * (1.0f + w);
        _model.state.gyroDynamicFilter[i].reconfigure(freq1, freq1, q);
        _model.state.gyroDynamicFilter2[i].reconfigure(freq2, freq2, q);
      } else {
        _model.state.gyroDynamicFilter[i].reconfigure(freq, freq, q);
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

    Math::Sma<VectorFloat, 8> _sma;

    Model& _model;
    Device::GyroDevice * _gyro;

#ifdef ESPFC_DSP
    Math::FFTAnalyzer<128> _fft[3];
#endif

};

}

}
#endif
