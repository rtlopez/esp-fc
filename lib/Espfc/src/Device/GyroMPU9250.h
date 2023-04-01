#ifndef _ESPFC_DEVICE_GYRO_MPU9250_H_
#define _ESPFC_DEVICE_GYRO_MPU9250_H_

#include "BusDevice.h"
#include "GyroMPU6050.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define MPU9250_USER_CTRL         0x6A
#define MPU9250_I2C_MST_EN        0x20
#define MPU9250_I2C_MST_400       0x0D
#define MPU9250_I2C_MST_500       0x09
#define MPU9250_I2C_MST_CTRL      0x24
#define MPU9250_I2C_MST_RESET     0x02

#define MPU9250_ACCEL_CONF2       0x1D

namespace Espfc {

namespace Device {

class GyroMPU9250: public GyroMPU6050
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, MPU6050_DEFAULT_ADDRESS);
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
      //_bus->writeByte(_addr, MPU9250_USER_CTRL, MPU9250_I2C_MST_RESET);

      // disable I2C master to reset slave registers allocation
      _bus->writeByte(_addr, MPU9250_USER_CTRL, 0);
      //delay(100);

      setDLPFMode(GYRO_DLPF_188);
      setRate(9); // 1000 / (9+1) = 100hz for mag, will be overwritten in GyroSensor
      delay(100);

      // enable I2C master mode
      _bus->writeByte(_addr, MPU9250_USER_CTRL, MPU9250_I2C_MST_EN);
      //delay(100);

      // set the I2C bus speed to 400 kHz
      _bus->writeByte(_addr, MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_400);
      //delay(100);

      return 1;
    }

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
      _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
      //Serial.print("MPU9250 "); Serial.print(_addr, HEX); Serial.print(' '); Serial.println(whoami, HEX);
      //D("mpu9250_whoami", _addr, whoami);
      return whoami == 0x71 || whoami == 0x73;
    }
};

}

}

#endif
