#ifndef _ESPFC_GYRO_DEVICE_H_
#define _ESPFC_GYRO_DEVICE_H_

#include "helper_3dmath.h"

namespace Espfc {

namespace Device {

class GyroDevice
{
  public:
    virtual int init();
    virtual int readGyro(VectorInt16& v);
    virtual int readAccel(VectorInt16& v);
};

}

}

#endif
