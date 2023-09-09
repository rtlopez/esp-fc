#ifndef _ESPFC_DEVICE_GYRO_BMI160_H_
#define _ESPFC_DEVICE_GYRO_BMI160_H_

#include "BusDevice.h"
#include "GyroDevice.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define BMI160_ADDRESS_FIRST      0x69
#define BMI160_ADDRESS_SECOND     0x68
#define BMI160_RA_CHIP_ID           0x00
#define BMI160_CHIP_ID_DEFAULT_VALUE 0xD1

#define BMI160_RA_GYRO_X_L          0x0C
#define BMI160_RA_GYRO_X_H          0x0D
#define BMI160_RA_GYRO_Y_L          0x0E
#define BMI160_RA_GYRO_Y_H          0x0F
#define BMI160_RA_GYRO_Z_L          0x10
#define BMI160_RA_GYRO_Z_H          0x11
#define BMI160_RA_ACCEL_X_L         0x12
#define BMI160_RA_ACCEL_X_H         0x13
#define BMI160_RA_ACCEL_Y_L         0x14
#define BMI160_RA_ACCEL_Y_H         0x15
#define BMI160_RA_ACCEL_Z_L         0x16
#define BMI160_RA_ACCEL_Z_H         0x17

#define BMI160_RA_ACCEL_CONF        0X40
#define BMI160_RA_ACCEL_RANGE       0X41

#define BMI160_RA_GYRO_CONF         0X42
#define BMI160_RA_GYRO_RANGE        0X43

#define BMI160_CMD_START_FOC        0x03
#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_CMD_GYR_MODE_NORMAL  0x15
#define BMI160_CMD_FIFO_FLUSH       0xB0
#define BMI160_CMD_INT_RESET        0xB1
#define BMI160_CMD_STEP_CNT_CLR     0xB2
#define BMI160_CMD_SOFT_RESET       0xB6

#define BMI160_RA_CMD               0x7E

namespace Espfc {

namespace Device {

class GyroBMI160: public GyroDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, BMI160_ADDRESS_FIRST) ? 1 : begin(bus, BMI160_ADDRESS_SECOND) ? 1 : 0;
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      // reset device
      _bus->writeByte(_addr, BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);
      delay(1);

      /* Issue a dummy-read to force the device into SPI comms mode */
      uint8_t dummy = 0;
      _bus->readByte(_addr, 0x7F, &dummy);
      delay(1);

      // Start up accelerometer
      _bus->writeByte(_addr, BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL);
      delay(100);

      // Start up gyroscope
      _bus->writeByte(_addr, BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);
      delay(100);

      // // Set odr and ranges
      // // Set acc_us = 0 & acc_bwp = 0b001 for high performance and OSR2 mode
      // _bus->writeByte(_addr, BMI160_RA_ACCEL_CONF, 0x00 | 0x10 | 0x0B);
      // delay(1);

      // // Set up full scale Accel range. +-16G
      _bus->writeByte(_addr, BMI160_RA_ACCEL_RANGE, 0x0C);
      delay(1);

      // // Set up full scale Gyro range. +-2000dps
      _bus->writeByte(_addr, BMI160_RA_GYRO_RANGE, 0x00);
      delay(1);

      // Set Accel ODR to 400hz, BWP mode to Oversample 4, LPF of ~40.5hz
      _bus->writeByte(_addr, BMI160_RA_ACCEL_CONF, 0x0A);
      delay(1);

      // Set Gyro ODR to 400hz, BWP mode to Oversample 4, LPF of ~34.15hz
      _bus->writeByte(_addr, BMI160_RA_GYRO_CONF, 0x0A);
      delay(1);

      return 1;
    }

    GyroDeviceType getType() const override
    {
      return GYRO_BMI160;
    }

    int readGyro(VectorInt16& v) override
    {
      uint8_t buffer[6];

      _bus->readFast(_addr, BMI160_RA_GYRO_X_L, 6, buffer);

      v.x = (((int16_t)buffer[1]) << 8) | buffer[0];
      v.y = (((int16_t)buffer[3]) << 8) | buffer[2];
      v.z = (((int16_t)buffer[5]) << 8) | buffer[4];

      return 1;
    }

    int readAccel(VectorInt16& v) override
    {
      uint8_t buffer[6];

      _bus->readFast(_addr, BMI160_RA_ACCEL_X_L, 6, buffer);

      v.x = (((int16_t)buffer[1]) << 8) | buffer[0];
      v.y = (((int16_t)buffer[3]) << 8) | buffer[2];
      v.z = (((int16_t)buffer[5]) << 8) | buffer[4];

      return 1;
    }

    void setDLPFMode(uint8_t mode) override
    {
    }

    int getRate() const override
    {
      return 8000;
    }

    void setRate(int rate) override
    {
    }

    void setFullScaleGyroRange(uint8_t range) override
    {
    }

    void setFullScaleAccelRange(uint8_t range) override
    {
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, BMI160_RA_CHIP_ID, &whoami);
      //D("bmi160:whoami", _addr, whoami);
      return whoami == BMI160_CHIP_ID_DEFAULT_VALUE;
    }

    void setSleepEnabled(bool enabled)
    {
    }

    void setClockSource(uint8_t source)
    {
    }
};

}

}

#endif
