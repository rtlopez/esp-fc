#ifndef _ESPFC_SENSOR_H_
#define _ESPFC_SENSOR_H_

#include "Arduino.h"
#include <MPU6050.h>
#include <HMC5883L.h>
#include <Adafruit_BMP280.h>

#include "Model.h"

#define ESPFC_FUZZY_ACCEL_ZERO 0.05
#define ESPFC_FUZZY_GYRO_ZERO 0.20

namespace Espfc {

class Sensor
{
  public:
    Sensor(Model& model): _model(model) {}
    int begin()
    {
      initGyro();

      //_mag.initialize();
      //_mag.setDataRate(HMC5883L_RATE_75);
      //_mag.setSampleAveraging(1);
      //_pressure.begin();
    }

    int update()
    {
      uint8_t buf[FIFO_SIZE];
      bool fetched = false;
      uint32_t now = millis();

      // too early, do not read
      if(_model.state.timestamp + _model.state.gyroSampleInterval > now) return 0;

      if(_model.config.gyroFifo)
      {
        int fifoCount = _gyro.getFIFOCount();
        if(fifoCount < FIFO_SIZE) return 0;
        int numSamples = fifoCount / FIFO_SIZE;

        // adjust timestamp for delayed samples
        if(numSamples > 1)
        {
          now -= _model.state.gyroSampleInterval * (numSamples - 1);
        }

        //Serial.print("fifo count: "); Serial.print(fifoCount); Serial.println();

        _gyro.getFIFOBytes(buf, FIFO_SIZE);
        toVector(_model.state.accelRaw, buf);
        toVector(_model.state.gyroRaw, buf + 6);
        //_mag.getHeading(&_model.state.magRaw.x, &_model.state.magRaw.y, &_model.state.magRaw.z);
      }
      else
      {
        _gyro.getMotion6(&_model.state.accelRaw.x, &_model.state.accelRaw.y, &_model.state.accelRaw.z,
                         &_model.state.gyroRaw.x,  &_model.state.gyroRaw.y,  &_model.state.gyroRaw.z);
      }
      _model.state.timestamp = now;

      updateModel();
      updateGyroBias();

      return 1;
    }

  private:
    void updateModel()
    {
      _model.state.accel = (VectorFloat)_model.state.accelRaw * _model.state.accelScale;
      _model.state.gyro  = (VectorFloat)_model.state.gyroRaw * _model.state.gyroScale;
    }

    void updateGyroBias()
    {
      if(!_model.state.gyroBiasValid)
      {
        VectorFloat deltaAccel = _model.state.accel - _model.state.accelPrev;
        _model.state.accelPrev = _model.state.accel;

        if((deltaAccel.getMagnitude() < ESPFC_FUZZY_ACCEL_ZERO) && (_model.state.gyro.getMagnitude() < ESPFC_FUZZY_GYRO_ZERO))
        {
          // what we are seeing on the gyros should be bias only so learn from this
          //_model.state.gyroBias.x = (1.0 - _model.state.gyroBiasAlpha) * _model.state.gyroBias.x + _model.state.gyroBiasAlpha * _model.state.gyro.x;
          //_model.state.gyroBias.y = (1.0 - _model.state.gyroBiasAlpha) * _model.state.gyroBias.y + _model.state.gyroBiasAlpha * _model.state.gyro.y;
          //_model.state.gyroBias.z = (1.0 - _model.state.gyroBiasAlpha) * _model.state.gyroBias.z + _model.state.gyroBiasAlpha * _model.state.gyro.z;
          _model.state.gyroBias = (_model.state.gyroBias * (1.0 - _model.state.gyroBiasAlpha)) + (_model.state.gyro * _model.state.gyroBiasAlpha);
          if(_model.state.gyroBiasSamples < (5 * _model.config.gyroSampleRate))
          {
              _model.state.gyroBiasSamples++;
              if(_model.state.gyroBiasSamples == (5 * _model.config.gyroSampleRate))
              {
                _model.state.gyroBiasValid = true;
              }
          }
        }
      }
      _model.state.gyro -= _model.state.gyroBias;
    }

    void toVector(VectorInt16& v, uint8_t * buf)
    {
      v.x = ((uint16_t)buf[0] << 8) | buf[1];
      v.y = ((uint16_t)buf[2] << 8) | buf[3];
      v.z = ((uint16_t)buf[4] << 8) | buf[5];
    }

    void initGyro()
    {
      _gyro.initialize();

      Serial.print("gyro init: "); Serial.print(_gyro.getDeviceID()); Serial.print(' '); Serial.println(_gyro.testConnection());

      _gyro.setFullScaleGyroRange(_model.config.gyroFsr);
      _gyro.setFullScaleAccelRange(_model.config.accelFsr);
      _gyro.setDLPFMode(_model.config.gyroDlpf);

      setSampleRate();
      setGyroScale();
      setAccelScale();

      // setup fifo
      if(_model.config.gyroFifo)
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
      int clock = 1000;
      if(_model.config.gyroDlpf == DLPF_256) clock = 8000;
      int r = clock / (_model.config.gyroSampleRate + 1);
      _model.config.gyroSampleRate = clock / (r + 1); // update to real sample rate
      _model.state.gyroSampleInterval = 1000 / _model.config.gyroSampleRate;

      _model.state.gyroBiasAlpha = 2.0f / _model.config.gyroSampleRate;
      _model.state.gyroBiasSamples = 0;

      Serial.print("gyro rate: "); Serial.print(r); Serial.print(' '); Serial.print(_model.state.gyroSampleInterval); Serial.print(' '); Serial.print(_model.config.gyroSampleRate); Serial.println();
      _gyro.setRate(r);
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
    }

    static const uint8_t FIFO_SIZE = 12;

    Model& _model;
    MPU6050 _gyro;
    HMC5883L _mag;
    Adafruit_BMP280 _pressure;
};

}

#endif
