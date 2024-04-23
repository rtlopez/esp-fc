#ifndef _ESPFC_DEVICE_GYRO_LSM6DSO_H_
#define _ESPFC_DEVICE_GYRO_LSM6DSO_H_

#include "BusDevice.h"
#include "GyroDevice.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

// https://github.com/arduino-libraries/Arduino_LSM6DSOX/blob/master/src/LSM6DSOX.cpp
#define LSM6DSOX_ADDRESS_FIRST     0x6A
#define LSM6DSOX_ADDRESS_SECOND    0x6b

// registers
#define LSM6DSO_REG_WHO_AM_I       0x0F
#define LSM6DSO_REG_CTRL1_XL       0x10
#define LSM6DSO_REG_CTRL2_G        0x11
#define LSM6DSO_REG_CTRL3_C        0x12
#define LSM6DSO_REG_CTRL4_C        0x13
#define LSM6DSO_REG_CTRL5_C        0x14
#define LSM6DSO_REG_CTRL6_C        0x15
#define LSM6DSO_REG_CTRL7_G        0x16
#define LSM6DSO_REG_CTRL8_XL       0x17
#define LSM6DSO_REG_CTRL9_XL       0x18
#define LSM6DSO_REG_CTRL10_C       0x19
#define LSM6DSO_REG_STATUS         0x1E
#define LSM6DSO_REG_OUTX_L_G       0x22
#define LSM6DSO_REG_OUTX_L_XL      0x28

// values
#define LSM6DSO_VAL_INT1_CTRL              0x02  // enable gyro data ready interrupt pin 1
#define LSM6DSO_VAL_INT2_CTRL              0x02  // enable gyro data ready interrupt pin 2
#define LSM6DSO_VAL_CTRL1_XL_ODR833        0x07  // accelerometer 833hz output data rate (gyro/8)
#define LSM6DSO_VAL_CTRL1_XL_ODR1667       0x08  // accelerometer 1666hz output data rate (gyro/4)
#define LSM6DSO_VAL_CTRL1_XL_ODR3332       0x09  // accelerometer 3332hz output data rate (gyro/2)
#define LSM6DSO_VAL_CTRL1_XL_ODR3333       0x0A  // accelerometer 6664hz output data rate (gyro/1)
#define LSM6DSO_VAL_CTRL1_XL_8G            0x03  // accelerometer 8G scale
#define LSM6DSO_VAL_CTRL1_XL_16G           0x01  // accelerometer 16G scale
#define LSM6DSO_VAL_CTRL1_XL_LPF1          0x00  // accelerometer output from LPF1
#define LSM6DSO_VAL_CTRL1_XL_LPF2          0x01  // accelerometer output from LPF2
#define LSM6DSO_VAL_CTRL2_G_ODR6664        0x0A  // gyro 6664hz output data rate
#define LSM6DSO_VAL_CTRL2_G_ODR3332        0x09  // gyro 3332hz output data rate
#define LSM6DSO_VAL_CTRL2_G_2000DPS        0x03  // gyro 2000dps scale
#define LSM6DSO_VAL_CTRL3_C_BDU            0x40  // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
#define LSM6DSO_VAL_CTRL3_C_H_LACTIVE      0x00  // (bit 5) interrupt pins active high
#define LSM6DSO_VAL_CTRL3_C_PP_OD          0x00  // (bit 4) interrupt pins push/pull
#define LSM6DSO_VAL_CTRL3_C_SIM            0x00  // (bit 3) SPI 4-wire interface mode
#define LSM6DSO_VAL_CTRL3_C_IF_INC         0x04  // (bit 2) auto-increment address for burst reads
#define LSM6DSO_VAL_CTRL4_C_I2C_DISABLE    0x04  // (bit 2) disable I2C interface
#define LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G     0x02  // (bit 1) enable gyro LPF1
#define LSM6DSO_VAL_CTRL6_C_XL_HM_MODE     0x00  // (bit 4) enable accelerometer high performance mode
#define LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ    0x00  // (bits 2:0) gyro LPF1 cutoff 335.5hz
#define LSM6DSO_VAL_CTRL6_C_FTYPE_232HZ    0x01  // (bits 2:0) gyro LPF1 cutoff 232.0hz
#define LSM6DSO_VAL_CTRL6_C_FTYPE_171HZ    0x02  // (bits 2:0) gyro LPF1 cutoff 171.1hz
#define LSM6DSO_VAL_CTRL6_C_FTYPE_609HZ    0x03  // (bits 2:0) gyro LPF1 cutoff 609.0hz
#define LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE   0x02  // (bit 1) disable I3C interface

// masks
#define LSM6DSO_MASK_CTRL3_C       0x7C // 0b01111100
#define LSM6DSO_MASK_CTRL3_C_RESET 0x01 // 0b00000001
#define LSM6DSO_MASK_CTRL4_C       0x06 // 0b00000110
#define LSM6DSO_MASK_CTRL6_C       0x17 // 0b00010111
#define LSM6DSO_MASK_CTRL9_XL      0x02 // 0b00000010

namespace Espfc {

namespace Device {

class GyroLSM6DSO: public GyroDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, LSM6DSOX_ADDRESS_FIRST) ? 1 : begin(bus, LSM6DSOX_ADDRESS_SECOND) ? 1 : 0;
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      // reset device
      _bus->writeMask(_addr, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C_RESET, 1);
      delay(100);

      // Accel, 833hz ODR, 16G scale, use LPF1 output
      _bus->writeByte(_addr, LSM6DSO_REG_CTRL1_XL, (LSM6DSO_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSO_VAL_CTRL1_XL_16G << 2) | (LSM6DSO_VAL_CTRL1_XL_LPF1 << 1));
      delay(1);

      // Gyro, 6664hz ODR, 2000dps scale
      _bus->writeByte(_addr, LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2));
      delay(1);

      // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
      _bus->writeMask(_addr, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C, (LSM6DSO_VAL_CTRL3_C_BDU | LSM6DSO_VAL_CTRL3_C_H_LACTIVE | LSM6DSO_VAL_CTRL3_C_PP_OD | LSM6DSO_VAL_CTRL3_C_SIM | LSM6DSO_VAL_CTRL3_C_IF_INC));

      // enable accelerometer high performane mode; set gyro LPF1 cutoff to 335.5hz
      _bus->writeMask(_addr, LSM6DSO_REG_CTRL4_C, LSM6DSO_MASK_CTRL4_C, (LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G));

      // enable gyro LPF1
      _bus->writeMask(_addr, LSM6DSO_REG_CTRL6_C, LSM6DSO_MASK_CTRL6_C, (LSM6DSO_VAL_CTRL6_C_XL_HM_MODE | LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ));

      // disable I3C interface
      _bus->writeMask(_addr, LSM6DSO_REG_CTRL9_XL, LSM6DSO_MASK_CTRL9_XL, LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE);

      return 1;
    }

    GyroDeviceType getType() const override
    {
      return GYRO_LSM6DSO;
    }

    int FAST_CODE_ATTR readGyro(VectorInt16& v) override
    {
      int16_t buffer[3];

      _bus->readFast(_addr, LSM6DSO_REG_OUTX_L_G, 6, (uint8_t*)buffer);

      v.x = buffer[0];
      v.y = buffer[1];
      v.z = buffer[2];

      return 1;
    }

    int readAccel(VectorInt16& v) override
    {
      int16_t buffer[3];

      _bus->readFast(_addr, LSM6DSO_REG_OUTX_L_XL, 6, (uint8_t*)buffer);

      v.x = buffer[0];
      v.y = buffer[1];
      v.z = buffer[2];

      return 1;
    }

    void setDLPFMode(uint8_t mode) override
    {
    }

    int getRate() const override
    {
      return 6664;
    }

    void setRate(int rate) override
    {
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, LSM6DSO_REG_WHO_AM_I, &whoami);
      //D("lsm6dso:whoami", _addr, whoami);
      return whoami == 0x6C || whoami == 0x69;
    }
};

}

}

#endif
