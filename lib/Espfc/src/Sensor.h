#ifndef _ESPFC_SENSOR_H_
#define _ESPFC_SENSOR_H_

#include "Arduino.h"
#include <MPU6050.h>
#include <HMC5883L.h>
#include <Adafruit_BMP280.h>

#include "Model.h"
#include "Filter.h"
#include "Fusion.h"

#define ESPFC_FUZZY_ACCEL_ZERO 0.05
#define ESPFC_FUZZY_GYRO_ZERO 0.20

namespace Espfc {

class Sensor
{
  public:
    Sensor(Model& model): _model(model), _fusion(model) {}
    int begin()
    {
      initGyro();
      initMag();
      initPresure();
      initFilter();
      _fusion.begin();
    }

    int update()
    {
      _model.state.stats.start(COUNTER_GYRO_READ);
      int ret = readSensors();
      _model.state.stats.end(COUNTER_GYRO_READ);
      if(!ret) return 0;

      _model.state.stats.start(COUNTER_GYRO_FILTER);
      updateSensors();
      _model.state.stats.end(COUNTER_GYRO_FILTER);

      if(_model.config.fusionDelay)
      {
        // if fusion is delayed, update attitude using current gyro reading
        _model.state.angle += _model.state.gyro * _model.state.gyroDt;
      }
      else
      {
        _fusion.update();
      }

      return 1;
    }

    int updateDelayed()
    {
      if(_model.config.accelMode == ACCEL_DELAYED)
      {
        _model.state.stats.start(COUNTER_ACCEL_READ);
        readAccel();
        _model.state.stats.end(COUNTER_ACCEL_READ);
        _model.state.stats.start(COUNTER_ACCEL_FILTER);
        updateAccel();
        _model.state.stats.end(COUNTER_ACCEL_FILTER);
      }

      if(_model.config.magEnable)
      {
        _model.state.stats.start(COUNTER_MAG_READ);
        int ret = readMag();
        _model.state.stats.end(COUNTER_MAG_READ);
        _model.state.stats.start(COUNTER_MAG_FILTER);
        updateMag();
        _model.state.stats.end(COUNTER_MAG_FILTER);
      }

      if(_model.config.fusionDelay)
      {
        _fusion.update();
      }
    }

  private:
    int readSensors()
    {
      switch(_model.config.accelMode)
      {
        case ACCEL_GYRO_FIFO:
          return readGyroAccelFifo();
        case ACCEL_GYRO:
          return readGyroAccel();
        case ACCEL_NONE:
        case ACCEL_DELAYED:
        default:
          return readGyro();
      }
    }

    void updateSensors()
    {
      switch(_model.config.accelMode)
      {
        case ACCEL_GYRO_FIFO:
        case ACCEL_GYRO:
          updateAccel();
        case ACCEL_NONE:
        case ACCEL_DELAYED:
        default:
          updateGyro();
      }
    }

    int readGyroAccelFifo()
    {
      uint8_t buf[FIFO_SIZE];
      int numSamples = 0;
      do
      {
        int fifoCount = _gyro.getFIFOCount();
        if(fifoCount < FIFO_SIZE) return 0;
        numSamples = fifoCount / FIFO_SIZE;
        _gyro.getFIFOBytes(buf, FIFO_SIZE);
      }
      while(numSamples > 1); // discard late samples and use only latest
      toVector(_model.state.accelRaw, buf);
      toVector(_model.state.gyroRaw, buf + 6);
      return 1;
    }

    int readGyroAccel()
    {
      _gyro.getMotion6(
        &_model.state.accelRaw.x, &_model.state.accelRaw.y, &_model.state.accelRaw.z,
        &_model.state.gyroRaw.x,  &_model.state.gyroRaw.y,  &_model.state.gyroRaw.z
      );
      return 1;
    }

    int readGyro()
    {
      _gyro.getRotation(&_model.state.gyroRaw.x,  &_model.state.gyroRaw.y,  &_model.state.gyroRaw.z);
      return 1;
    }

    int readAccel()
    {
      _gyro.getAcceleration(&_model.state.accelRaw.x, &_model.state.accelRaw.y, &_model.state.accelRaw.z);
      return 1;
    }

    int readMag()
    {
      if(_model.config.magEnable && _model.state.magTimestamp + _model.state.magSampleInterval < _model.state.gyroTimestamp)
      {
        _mag.getHeading(&_model.state.magRaw.x, &_model.state.magRaw.y, &_model.state.magRaw.z);
        _model.state.magTimestamp = _model.state.gyroTimestamp;
        return 1;
      }
      return 0;
    }

    void updateGyro()
    {
      _model.state.gyroScaled  = (VectorFloat)_model.state.gyroRaw  * _model.state.gyroScale;
      for(size_t i; i < 3; ++i)
      {
        _model.state.gyroScaled.set(i, Math::deadband(_model.state.gyroScaled[i], _model.state.gyroDeadband));
      }
      for(size_t i = 0; i < 3; i++)
      {
        _model.state.gyro.set(i, _model.state.gyroFilter[i].update(_model.state.gyroScaled[i]));
      }

      /*
      if(!_model.state.gyroBiasValid)
      {
        VectorFloat deltaAccel = _model.state.accel - _model.state.accelPrev;
        _model.state.accelPrev = _model.state.accel;
        if(deltaAccel.getMagnitude() < ESPFC_FUZZY_ACCEL_ZERO && _model.state.gyro.getMagnitude() < ESPFC_FUZZY_GYRO_ZERO)
        {
          // what we are seeing on the gyros should be bias only so learn from this
          _model.state.gyroBias = (_model.state.gyroBias * (1.0 - _model.state.gyroBiasAlpha)) + (_model.state.gyro * _model.state.gyroBiasAlpha);
          if(_model.state.gyroBiasSamples < (5 * _model.state.gyroSampleRate))
          {
              _model.state.gyroBiasSamples++;
              if(_model.state.gyroBiasSamples == (5 * _model.state.gyroSampleRate))
              {
                _model.state.gyroBiasValid = true;
              }
          }
        }
      }
      */

      if(_model.state.gyroBiasSamples)
      {
        VectorFloat deltaAccel = _model.state.accel - _model.state.accelPrev;
        _model.state.accelPrev = _model.state.accel;
        if(deltaAccel.getMagnitude() < ESPFC_FUZZY_ACCEL_ZERO &&
          _model.state.gyro.getMagnitude() < ESPFC_FUZZY_GYRO_ZERO)
        {
          _model.state.gyroBias = (_model.state.gyroBias * (1.0 - _model.state.gyroBiasAlpha)) + (_model.state.gyro * _model.state.gyroBiasAlpha);
          _model.state.gyroBiasSamples--;
          if(_model.state.gyroBiasSamples == 0)
          {
            _model.state.gyroBiasValid = true;
          }
        }
      }

      _model.state.gyro -= _model.state.gyroBias;
    }

    void updateAccel()
    {
      _model.state.accelScaled = (VectorFloat)_model.state.accelRaw * _model.state.accelScale;
      for(size_t i = 0; i < 3; i++)
      {
        _model.state.accel.set(i, _model.state.accelFilter[i].update(_model.state.accelScaled[i]));
      }

      if(_model.state.accelBiasSamples)
      {
        _model.state.accelBias = (_model.state.accelBias * (1.0f - _model.state.accelBiasAlpha)) + (_model.state.accel * _model.state.accelBiasAlpha);
        _model.state.accelBiasSamples--;
        if(_model.state.accelBiasSamples == 0)
        {
          _model.state.accelBias.z -= 1.0f;
          _model.state.accelBiasValid = true;
        }
      }
      else
      {
        _model.state.accel -= _model.state.accelBias;
      }
    }

    void updateMag()
    {
      if(_model.config.magEnable)
      {
        _model.state.magScaled  = (VectorFloat)_model.state.magRaw  * _model.state.magScale;
        for(size_t i = 0; i < 3; i++)
        {
          _model.state.mag.set(i, _model.state.magFilter[i].update(_model.state.magScaled[i]));
        }
        if(_model.state.magCalibrationValid && _model.state.magCalibration == 0)
        {
          _model.state.mag -= _model.config.magCalibrationOffset;
          _model.state.mag *= _model.config.magCalibrationScale;
        }
        collectMagCalibration();
      }
    }

    void collectMagCalibration()
    {
      if(!_model.config.magEnable) return;
      if(_model.state.magCalibration == 1)
      {
        resetMagCalibration();
        _model.state.magCalibration = 2;
      }

      if(_model.state.magCalibration == 0) return;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationData[i][0] = _model.state.mag.get(i) < _model.state.magCalibrationData[i][0] ? _model.state.mag.get(i) : _model.state.magCalibrationData[i][0];
        _model.state.magCalibrationData[i][1] = _model.state.mag.get(i) > _model.state.magCalibrationData[i][1] ? _model.state.mag.get(i) : _model.state.magCalibrationData[i][1];
      }
      updateMagCalibration();
    }

    void resetMagCalibration()
    {
      if(!_model.config.magEnable) return;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationData[i][0] = 0;
        _model.state.magCalibrationData[i][1] = 0;
      }
      updateMagCalibration();
    }

    void updateMagCalibration()
    {
      if(!_model.config.magEnable) return;
      // just in case when the calibration data is not valid
      _model.state.magCalibrationValid = false;
      for(int i = 0; i < 3; i++)
      {
        _model.config.magCalibrationOffset.set(i, 0.f);
        _model.config.magCalibrationScale.set(i, 1.f);
      }

      // find biggest range
      float maxDelta = -1;
      for(int i = 0; i < 3; i++)
      {
        if((_model.state.magCalibrationData[i][1] - _model.state.magCalibrationData[i][0]) > maxDelta)
        {
          maxDelta = _model.state.magCalibrationData[i][1] - _model.state.magCalibrationData[i][0];
        }
      }

      if(maxDelta <= 0) return;
      const float epsilon = 0.001f;
      maxDelta /= 2;                                         // this is the max +/- range
      for (int i = 0; i < 3; i++)
      {
        float delta = (_model.state.magCalibrationData[i][1] - _model.state.magCalibrationData[i][0]) / 2.f;
        if(delta < epsilon && delta > -epsilon) return;
        float offset = (_model.state.magCalibrationData[i][1] + _model.state.magCalibrationData[i][0]) / 2.f;
        _model.config.magCalibrationScale.set(i, maxDelta / delta);     // makes everything the same range
        _model.config.magCalibrationOffset.set(i, offset);
      }
      _model.state.magCalibrationValid = true;
    }

    void toVector(VectorInt16& v, uint8_t * buf)
    {
      v.x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
      v.y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
      v.z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    }

    void initGyro()
    {
      _gyro.initialize();
      _model.logger.info().log(F("GYRO INIT")).log(_gyro.getDeviceID()).logln(_gyro.testConnection());

      setSampleRate();
      setGyroScale();
      setAccelScale();

      // setup fifo
      if(_model.config.accelMode == ACCEL_GYRO_FIFO)
      {
        _gyro.setAccelFIFOEnabled(true);
        _gyro.setXGyroFIFOEnabled(true);
        _gyro.setYGyroFIFOEnabled(true);
        _gyro.setZGyroFIFOEnabled(true);
        _gyro.resetFIFO();
        _gyro.setFIFOEnabled(true);
      }
    }

    void setSampleRate()
    {
      // sample rate = clock / ( divider + 1)
      int clock = 1000;
      if(_model.config.gyroDlpf == GYRO_DLPF_256) clock = 8000;
      int rate = -1;
      switch(_model.config.gyroSampleRate)
      {
        case GYRO_RATE_1000: rate = 1000; break;
        case GYRO_RATE_500: rate = 500; break;
        case GYRO_RATE_333: rate = 333; break;
        case GYRO_RATE_250: rate = 250; break;
        case GYRO_RATE_200: rate = 200; break;
        case GYRO_RATE_166: rate = 166; break;
        case GYRO_RATE_100: rate = 100; break;
        case GYRO_RATE_50:  rate =  50; break;
        default: rate = 100;
      }

      _model.state.gyroDivider = (clock / (rate + 1)) + 1;
      _model.state.gyroSampleRate = clock / (_model.state.gyroDivider); // update to real sample rate
      _model.state.gyroSampleInterval = (1000000 / _model.state.gyroSampleRate);

      _model.state.gyroBiasAlpha = 5.0f / _model.state.gyroSampleRate; // higher value gives faster calibration, was 2
      _model.state.gyroBiasSamples = 2 * _model.state.gyroSampleRate;

      _model.state.accelBiasAlpha = 5.0f / _model.state.gyroSampleRate; // higher value gives faster calibration, was 2
      _model.state.accelBiasSamples = 2 * _model.state.gyroSampleRate;

      _gyro.setDLPFMode(_model.config.gyroDlpf);
      _gyro.setRate(_model.state.gyroDivider - 1);

      _model.logger.info().log(F("GYRO RATE")).log(_model.state.gyroDivider).log(_model.state.gyroSampleRate).logln(_model.state.gyroSampleInterval);
    }

    void setGyroScale()
    {
      switch(_model.config.gyroFsr)
      {
        case GYRO_FS_2000: _model.state.gyroScale = M_PI /  (16.384 * 180.0); break;
        case GYRO_FS_1000: _model.state.gyroScale = M_PI /  (32.768 * 180.0); break;
        case GYRO_FS_500:  _model.state.gyroScale = M_PI /  (65.535 * 180.0); break;
        case GYRO_FS_250:  _model.state.gyroScale = M_PI / (131.072 * 180.0); break;
      }
      _gyro.setFullScaleGyroRange(_model.config.gyroFsr);
    }

    void setAccelScale()
    {
      switch(_model.config.accelFsr)
      {
        case ACCEL_FS_16: _model.state.accelScale = 1.0 /  2048.0; break;
        case ACCEL_FS_8:  _model.state.accelScale = 1.0 /  4096.0; break;
        case ACCEL_FS_4:  _model.state.accelScale = 1.0 /  8192.0; break;
        case ACCEL_FS_2:  _model.state.accelScale = 1.0 / 16384.0; break;
      }
      _gyro.setFullScaleAccelRange(_model.config.accelFsr);
    }

    void initMag()
    {
      if(!_model.config.magEnable) return;
      _mag.initialize();

      _model.logger.info().log(F("MAG INIT")).logln(_mag.testConnection());

      _mag.setSampleAveraging(_model.config.magAvr);
      _mag.setMode(HMC5883L_MODE_CONTINUOUS);

      setMagSampleRate();
      setMagScale();
    }

    void setMagSampleRate()
    {
      int rate = -1;
      switch(_model.config.magSampleRate)
      {
        case MAG_RATE_3:    rate = 3; break;
        case MAG_RATE_7P5:  rate = 7; break;
        case MAG_RATE_15:   rate = 15; break;
        case MAG_RATE_30:   rate = 30; break;
        case MAG_RATE_75:   rate = 75; break;
        default: _model.config.magSampleRate = MAG_RATE_15; rate = 15; return;
      }
      _model.state.magSampleRate = rate;
      _model.state.magSampleInterval = 1000000 / rate;
      _mag.setDataRate(_model.config.magSampleRate + 0x02);
      _model.logger.info().log(F("MAG RATE")).logln(_model.config.magSampleRate).log(_model.state.magSampleRate).logln(_model.state.magSampleInterval);
    }

    void setMagScale()
    {
      const float base = 1.0f;//1000.0;
      switch(_model.config.magFsr)
      {
        case MAG_GAIN_1370: _model.state.magScale = base / 1370; break;
        case MAG_GAIN_1090: _model.state.magScale = base / 1090; break;
        case MAG_GAIN_820:  _model.state.magScale = base / 820; break;
        case MAG_GAIN_660:  _model.state.magScale = base / 660; break;
        case MAG_GAIN_440:  _model.state.magScale = base / 440; break;
        case MAG_GAIN_390:  _model.state.magScale = base / 390; break;
        case MAG_GAIN_330:  _model.state.magScale = base / 330; break;
        case MAG_GAIN_220:  _model.state.magScale = base / 220; break;
      }
      _mag.setGain(_model.config.magFsr);
    }

    void initPresure()
    {
      //_baro.begin();
    }

    void initFilter()
    {
      for(size_t i = 0; i < 3; i++)
      {
        _model.state.gyroFilter[i].begin((FilterType)_model.config.gyroFilterType, _model.config.gyroFilterCutFreq, _model.state.gyroSampleRate);
        _model.state.accelFilter[i].begin((FilterType)_model.config.accelFilterType, _model.config.accelFilterCutFreq, _model.state.gyroSampleRate);
        _model.state.magFilter[i].begin((FilterType)_model.config.magFilterType, _model.config.magFilterCutFreq, _model.state.gyroSampleRate);
      }
    }

    static const uint8_t FIFO_SIZE = 12;

    Model& _model;
    Fusion _fusion;
    MPU6050 _gyro;
    HMC5883L _mag;
    Adafruit_BMP280 _baro;
};

}

#endif
