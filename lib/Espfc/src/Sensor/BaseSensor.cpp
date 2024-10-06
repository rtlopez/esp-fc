#include "BaseSensor.h"
#include "ModelConfig.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Sensor {

void FAST_CODE_ATTR BaseSensor::align(VectorFloat& dest, uint8_t rotation)
{
  const float x = dest.x;
  const float y = dest.y;
  const float z = dest.z;

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

void FAST_CODE_ATTR BaseSensor::toVector(VectorInt16& v, uint8_t * buf)
{
  v.x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
  v.y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
  v.z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
}

}

}
