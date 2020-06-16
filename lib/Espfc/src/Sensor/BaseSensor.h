#ifndef _ESPFC_SENSOR_BASE_SENSOR_H_
#define _ESPFC_SENSOR_BASE_SENSOR_H_

#define ESPFC_FUZZY_ACCEL_ZERO 0.05
#define ESPFC_FUZZY_GYRO_ZERO 0.20

#include "Model.h"
#include "Hardware.h"

#include <math.h>

namespace Espfc {

namespace Sensor {

class BaseSensor
{
  public:
    void align(VectorInt16& dest, uint8_t rotation) /* ICACHE_RAM_ATTR */
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

    void toVector(VectorInt16& v, uint8_t * buf)
    {
      v.x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
      v.y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
      v.z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    }
};

}

}

#endif