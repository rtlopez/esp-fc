#ifndef _ESPFC_GYRO_DEVICE_H_
#define _ESPFC_GYRO_DEVICE_H_

#include "helper_3dmath.h"
#include "BusDevice.h"

namespace Espfc {

enum GyroDeviceType {
  GYRO_AUTO    = 0,
  GYRO_NONE    = 1,
  GYRO_MPU6000 = 2,
  GYRO_MPU6050 = 3,
  GYRO_MPU6500 = 4,
  GYRO_MPU9250 = 5
};

namespace Device {

class GyroDevice
{
  public:
    virtual int begin(BusDevice * bus, uint8_t addr = -1) = 0;

    virtual GyroDeviceType getType() const = 0;

    virtual int readGyro(VectorInt16& v) = 0;
    virtual int readAccel(VectorInt16& v) = 0;

    virtual void setDLPFMode(uint8_t mode) = 0;
    virtual void setRate(uint8_t rate) = 0;
    virtual void setFullScaleGyroRange(uint8_t range) = 0;
    virtual void setFullScaleAccelRange(uint8_t range) = 0;

    virtual bool testConnection() = 0;

    static const char ** getNames()
    {
      static const char* devChoices[]   = { PSTR("AUTO"), PSTR("NONE"), PSTR("MPU6000"), PSTR("MPU6050"), PSTR("MPU6500"), PSTR("MPU9250"), NULL };
      return devChoices;
    }
};

}

}

#endif
