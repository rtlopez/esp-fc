#ifndef _ESPFC_DEVICE_GYRO_ICM20602_H_
#define _ESPFC_DEVICE_GYRO_ICM20602_H_

#include "BusDevice.h"
#include "GyroMPU6050.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define ICM20602_RA_ACCEL2_CONFIG     0x1D
#define ICM20602_WHOAMI_DEFAULT_VALUE 0x12

namespace Espfc {

namespace Device {

class GyroICM20602: public GyroMPU6050
{
  public:
    GyroDeviceType getType() const override
    {
      return GYRO_ICM20602;
    }

    void setDLPFMode(uint8_t mode) override
    {
      GyroMPU6050::setDLPFMode(mode);
      _bus->writeByte(_addr, ICM20602_RA_ACCEL2_CONFIG, mode);
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
      //D("icm20602:whoami", _addr, whoami);
      return whoami == ICM20602_WHOAMI_DEFAULT_VALUE;
    }
};

}

}

#endif
