#ifndef _ESPFC_DEVICE_GYRO_MPU6500_H_
#define _ESPFC_DEVICE_GYRO_MPU6500_H_

#include "BusDevice.h"
#include "GyroMPU6050.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define MPU6500_USER_CTRL         0x6A
#define MPU6500_I2C_MST_EN        0x20
#define MPU6500_I2C_IF_DIS        0x10
#define MPU6500_I2C_MST_400       0x0D
#define MPU6500_I2C_MST_500       0x09
#define MPU6500_I2C_MST_CTRL      0x24
#define MPU6500_I2C_MST_RESET     0x02

#define MPU6500_ACCEL_CONF2       0x1D

#define MPU6500_WHOAMI_DEFAULT_VALUE 0x70
#define MPU6555_WHOAMI_DEFAULT_VALUE 0x75

namespace Espfc {

namespace Device {

class GyroMPU6500: public GyroMPU6050
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, MPU6050_ADDRESS_FIRST) ? 1 : begin(bus, MPU6050_ADDRESS_SECOND) ? 1 : 0;
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      _bus->writeByte(_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_RESET);
      delay(100);

      setClockSource(MPU6050_CLOCK_PLL_XGYRO);

      setSleepEnabled(false);
      delay(100);      

      // reset I2C master
      //_bus->writeByte(_addr, MPU6500_USER_CTRL, MPU6500_I2C_MST_RESET);

      // disable I2C master to reset slave registers allocation
      _bus->writeByte(_addr, MPU6500_USER_CTRL, 0);
      //delay(100);

      // temporary force 1k sample rate for mag initiation, will be overwritten in GyroSensor
      setDLPFMode(GYRO_DLPF_188);
      setRate(9); // 1000 / (9+1) = 100hz
      delay(100);

      // enable I2C master mode, and disable I2C if SPI
      if(_bus->isSPI())
      {
        _bus->writeByte(_addr, MPU6500_USER_CTRL, MPU6500_I2C_MST_EN | MPU6500_I2C_IF_DIS);
      }
      else
      {
        _bus->writeByte(_addr, MPU6500_USER_CTRL, MPU6500_I2C_MST_EN);
      }
      //delay(100);

      // set the I2C bus speed to 400 kHz
      _bus->writeByte(_addr, MPU6500_I2C_MST_CTRL, MPU6500_I2C_MST_400);
      //delay(100);

      return 1;
    }

    GyroDeviceType getType() const override
    {
      return GYRO_MPU6500;
    }

    void setDLPFMode(uint8_t mode) override
    {
      GyroMPU6050::setDLPFMode(mode);
      _bus->writeByte(_addr, MPU6500_ACCEL_CONF2, mode);
    }


    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
      //D("MPU6500:whoami", _addr, whoami);
      return whoami == MPU6500_WHOAMI_DEFAULT_VALUE || whoami == MPU6555_WHOAMI_DEFAULT_VALUE;
    }
};

}

}

#endif
