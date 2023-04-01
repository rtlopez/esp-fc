#ifndef _ESPFC_DEVICE_GYRO_MPU6050_H_
#define _ESPFC_DEVICE_GYRO_MPU6050_H_

#include "BusDevice.h"
#include "GyroDevice.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75


#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_PWR1_SLEEP_BIT          6

#define MPU6050_CLOCK_PLL_XGYRO         0x01

#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_RESET               0x80

#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2


namespace Espfc {

namespace Device {

class GyroMPU6050: public GyroDevice
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
      delay(15);

      setSleepEnabled(false);

      return 1;
    }

    GyroDeviceType getType() const override
    {
      return GYRO_MPU6050;
    }

    int readGyro(VectorInt16& v) override
    {
      uint8_t buffer[6];

      _bus->readFast(_addr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);

      v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
      v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
      v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

      return 1;
    }

    int readAccel(VectorInt16& v) override
    {
      uint8_t buffer[6];

      _bus->readFast(_addr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);

      v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
      v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
      v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

      return 1;
    }

    void setDLPFMode(uint8_t mode) override
    {
      _bus->writeBits(_addr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
    }

    void setRate(uint8_t rate) override
    {
      _bus->writeByte(_addr, MPU6050_RA_SMPLRT_DIV, rate);
    }

    void setFullScaleGyroRange(uint8_t range) override
    {
      _bus->writeBits(_addr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    }

    void setFullScaleAccelRange(uint8_t range) override
    {
      _bus->writeBits(_addr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, MPU6050_RA_WHO_AM_I, &whoami);
      //Serial.print("MPU6050 "); Serial.print(_addr, HEX); Serial.print(' '); Serial.println(whoami, HEX);
      //_bus->readBits(_addr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &whoami);
      //D("mpu6050_whoami", _addr, whoami);
      return whoami == 0x68 || whoami == 0x72;
    }

    void setSleepEnabled(bool enabled)
    {
      _bus->writeBit(_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
    }

    void setClockSource(uint8_t source)
    {
      _bus->writeBits(_addr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
    }
};

}

}

#endif
