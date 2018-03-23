#ifndef _ESPFC_GYRO_DEVICE_H_
#define _ESPFC_GYRO_DEVICE_H_

#include "helper_3dmath.h"

namespace Espfc {

namespace Device {

class GyroDevice
{
  public:
    virtual int readGyro(VectorInt16& v) = 0;
    virtual int readAccel(VectorInt16& v) = 0;

    virtual void setDLPFMode(uint8_t mode) = 0;
    virtual void setRate(uint8_t rate) = 0;
    virtual void setFullScaleGyroRange(uint8_t range) = 0;
    virtual void setFullScaleAccelRange(uint8_t range) = 0;

    virtual bool testConnection() = 0;
};

}

}

#endif
