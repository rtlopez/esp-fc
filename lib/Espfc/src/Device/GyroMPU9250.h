#ifndef _ESPFC_DEVICE_GYRO_MPU9250_H_
#define _ESPFC_DEVICE_GYRO_MPU9250_H_

#include "BusDevice.h"
#include "GyroMPU6050.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define MPU9250_ACCEL_CONF2          0x1D
#define MPU9250_WHOAMI_DEFAULT_VALUE 0x71
#define MPU9250_WHOAMI_ALT_VALUE     0x73

namespace Espfc {

namespace Device {

class GyroMPU9250: public GyroMPU6050
{
  public:
    GyroDeviceType getType() const override
    {
      return GYRO_MPU9250;
    }

    void setDLPFMode(uint8_t mode) override
    {
      GyroMPU6050::setDLPFMode(mode);
      _bus->writeByte(_addr, MPU9250_ACCEL_CONF2, mode);
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      uint8_t len = _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
      //D("mpu9250:whoami", _addr, whoami);
      return len == 1 && (whoami == MPU9250_WHOAMI_DEFAULT_VALUE || whoami == MPU9250_WHOAMI_ALT_VALUE);
    }
};

}

}

#endif
