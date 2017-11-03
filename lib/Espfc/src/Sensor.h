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
      _model.logger.info().log(F("SENSOR")).log(_model.config.accelMode).logln(_model.config.fusionDelay);
      initGyro();
      initMag();
      initPresure();
      _fusion.begin();
      _model.state.battery.timer.setRate(20);
      _model.state.battery.samples = 20;
      return 1;
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
        _model.state.angle += _model.state.gyro * _model.state.gyroTimer.delta;
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

      if(_model.config.magDev != MAG_NONE)
      {
        _model.state.stats.start(COUNTER_MAG_READ);
        readMag();
        _model.state.stats.end(COUNTER_MAG_READ);
        _model.state.stats.start(COUNTER_MAG_FILTER);
        updateMag();
        _model.state.stats.end(COUNTER_MAG_FILTER);
      }

      if(_model.config.fusionDelay)
      {
        _fusion.update();
      }

      if(_model.state.battery.timer.check())
      {
        updateBattery();
      }

      _model.finishCalibration();
      return 1;
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
        case ACCEL_OFF:
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
        case ACCEL_OFF:
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
      if(_model.config.magDev != MAG_NONE && _model.state.magTimer.check())
      {
        _mag.getHeading(&_model.state.magRaw.x, &_model.state.magRaw.y, &_model.state.magRaw.z);
        return 1;
      }
      return 0;
    }

    void updateGyro()
    {
      align(_model.state.gyroRaw, _model.config.gyroAlign);

      _model.state.gyro = (VectorFloat)_model.state.gyroRaw  * _model.state.gyroScale;
      for(size_t i = 0; i < 3; ++i)
      {
        if(_model.config.debugMode == DEBUG_NOTCH)
        {
          _model.state.debug[i] = lrintf(degrees(_model.state.gyro[i]));
        }
        _model.state.gyro.set(i, _model.state.gyroNotch1Filter[i].update(_model.state.gyro[i]));
        _model.state.gyro.set(i, _model.state.gyroNotch2Filter[i].update(_model.state.gyro[i]));
        if(_model.config.debugMode == DEBUG_GYRO)
        {
          _model.state.debug[i] = lrintf(degrees(_model.state.gyro[i]));
        }
        _model.state.gyro.set(i, _model.state.gyroFilter[i].update(_model.state.gyro[i]));
      }

      if(_model.state.gyroBiasSamples > 0)
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
            _model.state.buzzer.push(BEEPER_GYRO_CALIBRATED);
            _model.logger.info().log(F("GYRO CAL")).log(_model.state.gyroBias.x).log(_model.state.gyroBias.y).logln(_model.state.gyroBias.z);
          }
        }
      }
      _model.state.gyro -= _model.state.gyroBias;
    }

    void updateAccel()
    {
      align(_model.state.accelRaw, _model.config.accelAlign);

      _model.state.accel = (VectorFloat)_model.state.accelRaw * _model.state.accelScale;
      for(size_t i = 0; i < 3; i++)
      {
        _model.state.accel.set(i, _model.state.accelFilter[i].update(_model.state.accel[i]));
      }

      if(_model.state.accelBiasSamples > 0)
      {
        _model.state.accelBias = (_model.state.accelBias * (1.0f - _model.state.accelBiasAlpha)) + (_model.state.accel * _model.state.accelBiasAlpha);
        _model.state.accelBiasSamples--;
        if(_model.state.accelBiasSamples == 0)
        {
          _model.state.accelBias.z -= 1.0f;
          _model.logger.info().log(F("ACCEL CAL")).log(_model.state.accelBias.x).log(_model.state.accelBias.y).logln(_model.state.accelBias.z);
        }
      }
      else
      {
        _model.state.accel -= _model.state.accelBias;
      }
    }

    void updateMag()
    {
      if(_model.config.magDev != MAG_NONE)
      {
        align(_model.state.magRaw, _model.config.magAlign);
        _model.state.mag  = (VectorFloat)_model.state.magRaw  * _model.state.magScale;
        for(size_t i = 0; i < 3; i++)
        {
          _model.state.mag.set(i, _model.state.magFilter[i].update(_model.state.mag[i]));
        }
        if(_model.state.magCalibrationValid && _model.state.magCalibration == 0)
        {
          _model.state.mag -= _model.state.magCalibrationOffset;
          _model.state.mag *= _model.state.magCalibrationScale;
        }
        collectMagCalibration();
      }
    }

    void collectMagCalibration()
    {
      if(_model.config.magDev == MAG_NONE) return;
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
      if(_model.config.magDev == MAG_NONE) return;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationData[i][0] = 0;
        _model.state.magCalibrationData[i][1] = 0;
      }
      updateMagCalibration();
    }

    void updateMagCalibration()
    {
      if(_model.config.magDev == MAG_NONE) return;
      // just in case when the calibration data is not valid
      _model.state.magCalibrationValid = false;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationOffset.set(i, 0.f);
        _model.state.magCalibrationScale.set(i, 1.f);
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
        _model.state.magCalibrationScale.set(i, maxDelta / delta);     // makes everything the same range
        _model.state.magCalibrationOffset.set(i, offset);
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
      _model.logger.info().log(F("GYRO INIT")).log(_model.config.gyroDev).log(_gyro.getDeviceID()).logln(_gyro.testConnection());

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
      _gyro.setDLPFMode(_model.config.gyroDlpf);
      _gyro.setRate(_model.state.gyroDivider - 1);
      _model.logger.info().log(F("GYRO RATE")).log(_model.config.gyroDlpf).log(_model.state.gyroDivider).log(_model.state.gyroTimer.rate).logln(_model.state.gyroTimer.interval);
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
      if(_model.config.magDev == MAG_NONE) return;
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
      _mag.setDataRate(_model.config.magSampleRate + 0x02);
      _model.state.magTimer.setRate(rate);
      _model.logger.info().log(F("MAG RATE")).log(_model.config.magSampleRate).log(_model.state.magTimer.rate).logln(_model.state.magTimer.interval);
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

    void updateBattery()
    {
      // wemos d1 mini has divider 3.2:1 (220k:100k)
      const float alpha = 0.33f;
      float val = _model.state.battery.rawVoltage = analogRead(A0);
      val *= (int)_model.config.vbatScale;
      val /= 10.f;
      val *= _model.config.vbatResMult;
      val /= _model.config.vbatResDiv;
      val = Math::bound(val, 0.f, 255.f);
      val = (val * alpha + _model.state.battery.voltage * (1.f - alpha)); // smooth
      _model.state.battery.voltage = (uint8_t)lrintf(val);

      // cell count detection
      if(_model.state.battery.samples > 0)
      {
        _model.state.battery.cells = ((int)_model.state.battery.voltage + 40) / 42;  // round
        _model.state.battery.samples--;
      }
      _model.state.battery.cellVoltage = _model.state.battery.voltage / std::max((int)_model.state.battery.cells, 1);
    }

    void align(VectorInt16& dest, uint8_t rotation)
    {
      const int16_t x = dest.x;
      const int16_t y = dest.y;
      const int16_t z = dest.z;

      switch(rotation)
      {
        default:
        case ALIGN_CW0_DEG:
          dest.x = x;
          dest.y = y;
          dest.z = z;
          break;
        case ALIGN_CW90_DEG:
          dest.x = y;
          dest.y = -x;
          dest.z = z;
          break;
        case ALIGN_CW180_DEG:
          dest.x = -x;
          dest.y = -y;
          dest.z = z;
          break;
        case ALIGN_CW270_DEG:
          dest.x = -y;
          dest.y = x;
          dest.z = z;
          break;
        case ALIGN_CW0_DEG_FLIP:
          dest.x = -x;
          dest.y = y;
          dest.z = -z;
          break;
        case ALIGN_CW90_DEG_FLIP:
          dest.x = y;
          dest.y = x;
          dest.z = -z;
          break;
        case ALIGN_CW180_DEG_FLIP:
          dest.x = x;
          dest.y = -y;
          dest.z = -z;
          break;
        case ALIGN_CW270_DEG_FLIP:
          dest.x = -y;
          dest.y = -x;
          dest.z = -z;
          break;
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
