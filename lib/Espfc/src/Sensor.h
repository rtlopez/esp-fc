#ifndef _ESPFC_SENSOR_H_
#define _ESPFC_SENSOR_H_

#include <MPU6050.h>
#include <HMC5883L.h>
#include <Adafruit_BMP280.h>

#include "Model.h"

namespace Espfc {

class Sensor
{
  public:
    Sensor(Model& model): _model(model) {}
    int begin()
    {
      _gyro.initialize();
      _gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
      _gyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
      _gyro.setDLPFMode(MPU6050_DLPF_BW_188);
      _gyro.setRate(2); // (1000Hz / (2 + 1)) = 333Hz
      _gyro.setAccelFIFOEnabled(true);
      _gyro.setXGyroFIFOEnabled(true);
      _gyro.setYGyroFIFOEnabled(true);
      _gyro.setZGyroFIFOEnabled(true);

      _mag.initialize();
      _mag.setDataRate(HMC5883L_RATE_75);

      _pressure.begin();
    }

    int update()
    {
      read();
    }

  private:
   void read()
   {
     uint8_t buf[12];
     bool fetched = false;
     //_gyro.getMotion6(&_model.state.accelRaw.x, &_model.state.accelRaw.y, &_model.state.accelRaw.z,
     //                 &_model.state.gyroRaw.x, &_model.state.gyroRaw.y, &_model.state.gyroRaw.z);
     while(true)
     {
       int fifoCount = _gyro.getFIFOCount();
       if(fifoCount < 12) break;
       _gyro.getFIFOBytes(buf, 12);
       fetched = true;
     }
     if(fetched)
     {
       _model.state.accelRaw.x = ((uint16_t)buf[1] << 8) | buf[0];
       _model.state.accelRaw.y = ((uint16_t)buf[3] << 8) | buf[2];
       _model.state.accelRaw.z = ((uint16_t)buf[5] << 8) | buf[4];
       _model.state.gyroRaw.x  = ((uint16_t)buf[7] << 8) | buf[6];
       _model.state.gyroRaw.y  = ((uint16_t)buf[9] << 8) | buf[8];
       _model.state.gyroRaw.z = ((uint16_t)buf[11] << 8) | buf[10];
       _mag.getHeading(&_model.state.magRaw.x, &_model.state.magRaw.y, &_model.state.magRaw.z);
       _model.state.timestamp = millis();
     }
   }

    Model& _model;
    MPU6050 _gyro;
    HMC5883L _mag;
    Adafruit_BMP280 _pressure;
};

}

#endif
