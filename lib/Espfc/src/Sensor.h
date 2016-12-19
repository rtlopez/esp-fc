#ifndef _ESPFC_SENSOR_H_
#define _ESPFC_SENSOR_H_

#include <MPU6050.h>
#include <HMC5883L.h>
#include <Adafruit_BMP280.h>

namespace Espfc {

class Sensor
{
  public:
    Sensor() {}
    int begin()
    {
      _gyro.initialize();
      _mag.initialize();
      _pressure.begin();
    }
    int update() {}

  private:
    MPU6050 _gyro;
    HMC5883L _mag;
    Adafruit_BMP280 _pressure;
};

}

#endif
