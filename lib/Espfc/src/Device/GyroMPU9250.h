#ifndef _ESPFC_GYRO_MPU9250_H_
#define _ESPFC_GYRO_MPU9250_H_

#include "BusDevice.h"
#include "GyroMPU6050.h"
#include "helper_3dmath.h"
#include "Debug.h"

namespace Espfc {

namespace Device {

class GyroMPU9250: public GyroMPU6050
{
  public:
    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readBits(_addr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &whoami);
      D("mpu9250_whoami", _addr, whoami);
      return whoami == 0x71 || whoami == 0x73;
    }
};

}

}

#endif
