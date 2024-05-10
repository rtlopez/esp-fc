#ifndef _ESPFC_DEVICE_GYRO_BMI160_H_
#define _ESPFC_DEVICE_GYRO_BMI160_H_

#include "BusDevice.h"
#include "GyroDevice.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

#define BMI160_ADDRESS_FIRST        0x69
#define BMI160_ADDRESS_SECOND       0x68
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

#define BMI160_ACCEL_RATE_SEL_BIT    0
#define BMI160_ACCEL_RATE_SEL_LEN    4

#define BMI160_RA_ACCEL_CONF        0X40
#define BMI160_RA_ACCEL_RANGE       0X41

#define BMI160_GYRO_RATE_SEL_BIT    0
#define BMI160_GYRO_RATE_SEL_LEN    4

#define BMI160_RA_GYRO_CONF         0X42
#define BMI160_RA_GYRO_RANGE        0X43

#define BMI160_ACC_OFFSET_EN        6
#define BMI160_ACC_OFFSET_LEN       1
#define BMI160_GYR_OFFSET_EN        7
#define BMI160_GYR_OFFSET_LEN       1

#define BMI160_RA_OFFSET_0          0x71
#define BMI160_RA_OFFSET_1          0x72
#define BMI160_RA_OFFSET_2          0x73
#define BMI160_RA_OFFSET_3          0x74
#define BMI160_RA_OFFSET_4          0x75
#define BMI160_RA_OFFSET_5          0x76
#define BMI160_RA_OFFSET_6          0x77

#define BMI160_REG_INT_EN1          0x51
#define BMI160_INT_EN1_DRDY         0x10
#define BMI160_REG_INT_OUT_CTRL     0x53
#define BMI160_INT_OUT_CTRL_INT1_CONFIG 0x0A
#define BMI160_REG_INT_MAP1         0x56
#define BMI160_REG_INT_MAP1_INT1_DRDY 0x80

#define BMI160_CMD_START_FOC        0x03
#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_CMD_GYR_MODE_NORMAL  0x15
#define BMI160_CMD_FIFO_FLUSH       0xB0
#define BMI160_CMD_INT_RESET        0xB1
#define BMI160_CMD_STEP_CNT_CLR     0xB2
#define BMI160_CMD_SOFT_RESET       0xB6
#define BMI160_CMD_SPI_MODE         0x7F
#define BMI160_RESULT_OK            0x1

#define BMI160_RA_CMD               0x7E

namespace Espfc {

namespace Device {

class GyroBMI160: public GyroDevice
{
  public:
    enum {
        BMI160_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
        BMI160_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
        BMI160_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
        BMI160_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
    };

    enum {
        BMI160_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
        BMI160_GYRO_RANGE_1000,     /**<  +/- 1000 degrees/second */
        BMI160_GYRO_RANGE_500,      /**<  +/-  500 degrees/second */
        BMI160_GYRO_RANGE_250,      /**<  +/-  250 degrees/second */
        BMI160_GYRO_RANGE_125,      /**<  +/-  125 degrees/second */
    };

    enum {
        BMI160_ACCEL_RATE_25_2HZ = 5,  /**<   25/2  Hz */
        BMI160_ACCEL_RATE_25HZ,        /**<   25    Hz */
        BMI160_ACCEL_RATE_50HZ,        /**<   50    Hz */
        BMI160_ACCEL_RATE_100HZ,       /**<  100    Hz */
        BMI160_ACCEL_RATE_200HZ,       /**<  200    Hz */
        BMI160_ACCEL_RATE_400HZ,       /**<  400    Hz */
        BMI160_ACCEL_RATE_800HZ,       /**<  800    Hz */
        BMI160_ACCEL_RATE_1600HZ,      /**< 1600    Hz */
    };

    enum {
        BMI160_GYRO_RATE_25HZ = 6,     /**<   25    Hz */
        BMI160_GYRO_RATE_50HZ,         /**<   50    Hz */
        BMI160_GYRO_RATE_100HZ,        /**<  100    Hz */
        BMI160_GYRO_RATE_200HZ,        /**<  200    Hz */
        BMI160_GYRO_RATE_400HZ,        /**<  400    Hz */
        BMI160_GYRO_RATE_800HZ,        /**<  800    Hz */
        BMI160_GYRO_RATE_1600HZ,       /**< 1600    Hz */
        BMI160_GYRO_RATE_3200HZ,       /**< 3200    Hz */
    };

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

      if(_bus->isSPI())
      {
        /* 
          Issue a dummy-read to force the device into SPI comms mode 
          see https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf section 3.2.1
        */
        uint8_t dummy = 0;
        _bus->readByte(_addr, BMI160_CMD_SPI_MODE, &dummy);
        delay(100);
      }

      // Start up accelerometer
      _bus->writeByte(_addr, BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL);
      delay(100);

      // Start up gyroscope
      _bus->writeByte(_addr, BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);
      delay(100);

      // Set up full scale Accel range. +-16G
      _bus->writeByte(_addr, BMI160_RA_ACCEL_RANGE, BMI160_ACCEL_RANGE_16G);
      delay(1);

      // Set up full scale Gyro range. +-2000dps
      _bus->writeByte(_addr, BMI160_RA_GYRO_RANGE, BMI160_GYRO_RANGE_2000);
      delay(1);

      // Enable accel offset
      //_bus->writeBitsLsb(_addr, BMI160_RA_OFFSET_6, BMI160_ACC_OFFSET_EN, BMI160_ACC_OFFSET_LEN, BMI160_RESULT_OK);
      delay(1);

      // Enable gyro offset
      //_bus->writeBitsLsb(_addr, BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_EN, BMI160_GYR_OFFSET_LEN, BMI160_RESULT_OK);
      delay(1);

      // Enable data ready interrupt
      _bus->writeByte(_addr, BMI160_REG_INT_EN1, BMI160_INT_EN1_DRDY);
      delay(1);

      // Enable INT1 pin
      _bus->writeByte(_addr, BMI160_REG_INT_OUT_CTRL, BMI160_INT_OUT_CTRL_INT1_CONFIG);
      delay(1);

      // Map data ready interrupt to INT1 pin
      _bus->writeByte(_addr, BMI160_REG_INT_MAP1, BMI160_REG_INT_MAP1_INT1_DRDY);
      delay(1);

      // Set Accel rate 1600HZ
      _bus->writeByte(_addr, BMI160_RA_ACCEL_CONF, BMI160_ACCEL_RATE_800HZ);
      delay(1);

      // Set Gyro rate 3200HZ
      _bus->writeByte(_addr, BMI160_RA_GYRO_CONF, BMI160_GYRO_RATE_3200HZ);
      delay(1);

      return 1;
    }

    GyroDeviceType getType() const override
    {
      return GYRO_BMI160;
    }

    int FAST_CODE_ATTR readGyro(VectorInt16& v) override
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
      return 3200;
    }

    void setRate(int rate) override
    {
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, BMI160_RA_CHIP_ID, &whoami);
      //D("bmi160:whoami", _addr, whoami);
      return whoami == BMI160_CHIP_ID_DEFAULT_VALUE;
    }
};

}

}

#endif
