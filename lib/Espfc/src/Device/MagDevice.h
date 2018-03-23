#ifndef _ESPFC_MAG_DEVICE_H_
#define _ESPFC_MAG_DEVICE_H_

#include "helper_3dmath.h"

namespace Espfc {

namespace Device {

class MagDevice
{
  public:
    virtual int readMag(VectorInt16& v) = 0;

    virtual void setSampleAveraging(uint8_t averaging) = 0;
    virtual void setSampleRate(uint8_t rate) = 0;
    virtual void setMode(uint8_t mode) = 0;
    virtual void setGain(uint8_t scale) = 0;

    virtual bool testConnection() = 0;
};

}

}

#endif
