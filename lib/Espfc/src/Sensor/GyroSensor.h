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
    GyroSensor(Model& model): _dyn_notch_denom(1), _model(model) {}

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

      _sma.begin(_model.config.loopSync);
      _dyn_notch_denom = std::max((uint32_t)1, _model.state.loopTimer.rate / 1000);
      _dyn_notch_sma.begin(_dyn_notch_denom);

      for(size_t i = 0; i < 3; i++)
      {
#ifdef ESPFC_DSP
        _fft[i].begin(_model.state.loopTimer.rate / _dyn_notch_denom, _model.config.dynamicFilter, i);
#else
       _freqAnalyzer[i].begin(_model.state.loopTimer.rate / _dyn_notch_denom, _model.config.dynamicFilter);
#endif
      }

      _model.logger.info().log(F("GYRO INIT")).log(FPSTR(Device::GyroDevice::getName(_gyro->getType()))).log(_model.config.gyroDlpf).log(_gyro->getRate()).log(_model.state.gyroTimer.rate).logln(_model.state.gyroTimer.interval);

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

      if(_model.config.gyroFilter2.freq)
      {
        for(size_t i = 0; i < 3; ++i)
        {
          _model.state.gyroSampled.set(i, _model.state.gyroFilter2[i].update(input[i]));
        }
      } else {
        // moving average filter
        _model.state.gyroSampled = _sma.update(input);
      }

      return 1;
    }

    int filter()
    {
      if(!_model.gyroActive()) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_GYRO_FILTER);

      _model.state.gyro = _model.state.gyroSampled;

      calibrate();

      _model.state.gyroScaled = _model.state.gyro;

      // filtering
      for(size_t i = 0; i < 3; ++i)
      {
        if(_model.config.debugMode == DEBUG_GYRO_RAW)
        {
          _model.state.debug[i] = _model.state.gyroRaw[i];
        }
        if(_model.config.debugMode == DEBUG_GYRO_SCALED)
        {
          _model.state.debug[i] = lrintf(degrees(_model.state.gyroScaled[i]));
        }
        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == _model.config.debugAxis)
        {
          _model.state.debug[0] = lrintf(degrees(_model.state.gyro[i]));
        }

        _model.state.gyro.set(i, _model.state.gyroFilter3[i].update(_model.state.gyro[i]));

        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == _model.config.debugAxis)
        {
          _model.state.debug[1] = lrintf(degrees(_model.state.gyro[i]));
        }

        _model.state.gyro.set(i, _model.state.gyroNotch1Filter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroNotch2Filter[i].update(_model.state.gyro[i]));

        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == _model.config.debugAxis)
        {
          _model.state.debug[2] = lrintf(degrees(_model.state.gyro[i]));
        }

        _model.state.gyro.set(i, _model.state.gyroFilter[i].update(_model.state.gyro[i]));

        if(_model.config.debugMode == DEBUG_GYRO_SAMPLE && i == _model.config.debugAxis)
        {
          _model.state.debug[3] = lrintf(degrees(_model.state.gyro[i]));
        }
      }

      filterDynNotch();

      for(size_t i = 0; i < 3; ++i)
      {
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
    void filterDynNotch()
    {
      bool enabled = _model.isActive(FEATURE_DYNAMIC_FILTER);
      bool feed = _model.state.loopTimer.iteration % _dyn_notch_denom == 0;
      bool debug = _model.config.debugMode == DEBUG_FFT_FREQ || _model.config.debugMode == DEBUG_FFT_TIME;
      const float q = _model.config.dynamicFilter.q * 0.01;
      const size_t peakCount = _model.config.dynamicFilter.width;
      bool update = _model.state.dynamicFilterTimer.check();

      if(enabled || debug)
      {
        _model.state.gyroDynNotch = _dyn_notch_sma.update(_model.state.gyro);

        for(size_t i = 0; i < 3; ++i)
        {
#ifdef ESPFC_DSP
          (void)update;
          if(feed)
          {
            uint32_t startTime = micros();
            int status = _fft[i].update(_model.state.gyroDynNotch[i]);
            if(_model.config.debugMode == DEBUG_FFT_TIME)
            {
              if(i == 0) _model.state.debug[0] = status;
              _model.state.debug[i + 1] = micros() - startTime;
            }
            if(_model.config.debugMode == DEBUG_FFT_FREQ && i == _model.config.debugAxis)
            {
              _model.state.debug[0] = lrintf(_fft[i].peaks[0].freq);
              _model.state.debug[1] = lrintf(_fft[i].peaks[1].freq);
              _model.state.debug[2] = lrintf(_fft[i].peaks[2].freq);
              _model.state.debug[3] = lrintf(_fft[i].peaks[3].freq);
            }
            if(enabled && status)
            {
              for(size_t p = 0; p < peakCount; p++)
              {
                float freq = _fft[i].peaks[p].freq;
                if(freq >= _model.config.dynamicFilter.min_freq && freq <= _model.config.dynamicFilter.max_freq)
                {
                  _model.state.gyroDynNotchFilter[i][p].reconfigure(freq, freq, q);
                }
              }
            }
          }
#else
          if(feed)
          {
            uint32_t startTime = micros();
            _freqAnalyzer[i].update(_model.state.gyroDynNotch[i]);
            float freq = _freqAnalyzer[i].freq;
            if(_model.config.debugMode == DEBUG_FFT_TIME)
            {
              if(i == 0) _model.state.debug[0] = update;
              _model.state.debug[i + 1] = micros() - startTime;
            }
            if(_model.config.debugMode == DEBUG_FFT_FREQ)
            {
              _model.state.debug[i] = lrintf(freq);
              if(i == _model.config.debugAxis) _model.state.debug[3] = lrintf(degrees(_model.state.gyroDynNotch[i]));
            }
            if(enabled && update)
            {
              if(freq >= _model.config.dynamicFilter.min_freq && freq <= _model.config.dynamicFilter.max_freq)
              {
                for(size_t p = 0; p < peakCount; p++)
                {
                  size_t x = (p + i) % 3;
                  int harmonic = (p / 3) + 1;
                  _model.state.gyroDynNotchFilter[x][p].reconfigure(freq * harmonic, freq * harmonic, q);
                }
              }
            }
          }
#endif
          if(enabled)
          {
            for(size_t p = 0; p < peakCount; p++)
            {
              _model.state.gyro.set(i, _model.state.gyroDynNotchFilter[i][p].update(_model.state.gyro[i]));
            }
          }
        }
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
    Math::Sma<VectorFloat, 8> _dyn_notch_sma;
    size_t _dyn_notch_denom;

    Model& _model;
    Device::GyroDevice * _gyro;

#ifdef ESPFC_DSP
    Math::FFTAnalyzer<128> _fft[3];
#else
    Math::FreqAnalyzer _freqAnalyzer[3];
#endif

};

}

}
#endif
