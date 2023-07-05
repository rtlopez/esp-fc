#ifndef _ESPFC_SENSOR_GYRO_SENSOR_H_
#define _ESPFC_SENSOR_GYRO_SENSOR_H_

#include "BaseSensor.h"
#include "Device/GyroDevice.h"

#ifdef ESPFC_DSP
// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c
#include "dsps_fft4r.h"
#include "dsps_wind_hann.h"
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

      _idx = 0;
      _count = std::min(_model.config.loopSync, (int8_t)8);

#ifdef ESPFC_DSP
      dynamicFilterFFTInit();
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
        _model.state.gyro.set(i, _model.state.gyroNotch1Filter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroNotch2Filter[i].update(_model.state.gyro[i]));

        if(dynamicFilterEnabled || dynamicFilterDebug)
        {
#ifdef ESPFC_DSP
          if(dynamicFilterDebug && i == 0)
          {
            _model.state.debug[3] = _model.state.gyro[i];
          }
          _fft_in[i][_fft_c] = _model.state.gyro[i];
#else
          dynamicFilterAnalyze((Axis)i, dynamicFilterDebug);
#endif
          if(dynamicFilterUpdate) dynamicFilterApply((Axis)i);
          if(dynamicFilterEnabled)
          {
            _model.state.gyro.set(i, _model.state.gyroDynamicFilter[i].update(_model.state.gyro[i]));
            _model.state.gyro.set(i, _model.state.gyroDynamicFilter2[i].update(_model.state.gyro[i]));
          }
        }
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

#ifdef ESPFC_DSP
      dynamicFilterFFTAnalyze(dynamicFilterDebug);
#endif

      return 1;
    }

  private:
#ifdef ESPFC_DSP
    void dynamicFilterFFTInit()
    {
      _fft_c = 0;
      _fft_bucket_width = (float)_model.state.loopTimer.rate / N;

      dsps_fft4r_init_fc32(NULL, N >> 1);

      // Generate hann window
      dsps_wind_hann_f32(_fft_wind, N);
    }

    void dynamicFilterFFTAnalyze(bool debug)
    {
      if(++_fft_c < N) return;

      _fft_c = 0;

      const float loFreq = _model.config.dynamicFilter.min_freq;
      const float hiFreq = _model.config.dynamicFilter.max_freq;
      const float offset = _fft_bucket_width * 0.5f; // center of bucket

      for(size_t i = 0; i < 3; ++i)
      {
        // apply window
        for (size_t j = 0; j < N; j++)
        {
          _fft_in[i][j] *= _fft_wind[j]; // real
        }

        // FFT Radix-4
        dsps_fft4r_fc32(_fft_in[i], N >> 1);

        // Bit reverse 
        dsps_bit_rev4r_fc32(_fft_in[i], N >> 1);

        // Convert one complex vector with length N/2 to one real spectrum vector with length N/2
        dsps_cplx2real_fc32(_fft_in[i], N >> 1);

        // calculate magnitude
        for (size_t j = 0; j < N >> 1; j++)
        {
          _fft_out[i][j] = _fft_in[i][j * 2 + 0] * _fft_in[i][j * 2 + 0] + _fft_in[i][j * 2 + 1] * _fft_in[i][j * 2 + 1];
        }

        // TODO: find max noise freq
        float maxAmt = 0;
        float maxFreq = 0;
        for (size_t j = 1; j < (N >> 1) - 1; j++)
        {
          const float freq = _fft_bucket_width * j + offset;
          if(freq < loFreq) continue;
          if(freq > hiFreq) break;
          const float amt = _fft_out[i][j];
          if(amt > maxAmt)
          {
            maxAmt = amt;
            maxFreq = freq;
          }
        }
        _fft_max_freq[i] = maxFreq;

        if(debug)
        {
          _model.state.debug[i] = lrintf(maxFreq);
        }
      }     
    }
#endif

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
#ifdef ESPFC_DSP
      const float freq = _fft_max_freq[i];
#else
      const float freq = _model.state.gyroAnalyzer[i].freq;
#endif
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

#ifdef ESPFC_DSP
    static const size_t N = 128;
    size_t _fft_c;
    float _fft_bucket_width;
    float _fft_max_freq[3];

    // fft input and aoutput
    __attribute__((aligned(16))) float _fft_in[3][N];
    __attribute__((aligned(16))) float _fft_out[3][N];
    // Window coefficients
    __attribute__((aligned(16))) float _fft_wind[N];
#endif

};

}

}
#endif