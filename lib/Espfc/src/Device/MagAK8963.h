#ifndef _ESPFC_DEVICE_MAG_AK8963_H_
#define _ESPFC_DEVICE_MAG_AK8963_H_

#include "MagDevice.h"
#include "BusDevice.h"

#define MPU9250_ADDRESS           0x68
#define MPU9250_DEFAULT_ADDRESS   MPU9250_ADDRESS
#define MPU9250_I2C_SLV0_ADDR     0x25
#define MPU9250_I2C_SLV0_REG      0x26
#define MPU9250_I2C_SLV0_DO       0x63
#define MPU9250_I2C_SLV0_CTRL     0x27
#define MPU9250_I2C_SLV0_EN       0x80
#define MPU9250_WHO_AM_I          0x75
#define MPU9250_EXT_SENS_DATA_00  0x49

#define AK8963_ADDRESS            0x0C // this device only has one address
#define AK8963_DEFAULT_ADDRESS    AK8963_ADDRESS
#define AK8963_HXL                0x03
#define AK8963_CNTL1              0x0A
#define AK8963_PWR_DOWN           0x00
#define AK8963_CNT_MEAS1          0x12
#define AK8963_CNT_MEAS2          0x16
#define AK8963_FUSE_ROM           0x0F
#define AK8963_CNTL2              0x0B
#define AK8963_RESET              0x01
#define AK8963_ASA                0x10
#define AK8963_WHO_AM_I           0x00

namespace Espfc {

namespace Device {

class MagAK8963: public MagDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, AK8963_DEFAULT_ADDRESS, MPU9250_DEFAULT_ADDRESS);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      if(bus->getType() == BUS_I2C)
      {
        return begin(bus, AK8963_DEFAULT_ADDRESS, MPU9250_DEFAULT_ADDRESS);
      }
      else
      {
        return begin(bus, addr, addr); // for SPI addr is CS pin
      }
    }

    int begin(BusDevice * bus, uint8_t addr, uint8_t masterAddr) override
    {
      setBus(bus, addr, masterAddr);

      if(!testConnection()) return 0;

      // set AK8963 to Power Down
      writeSlave(AK8963_CNTL1, AK8963_PWR_DOWN);
      delay(100);

      // reset the AK8963
      writeSlave(AK8963_CNTL2, AK8963_RESET);
      delay(100);

      return 1;
    }

    int readMag(VectorInt16& v) override
    {
      uint8_t buffer[6];
      readMaster(MPU9250_EXT_SENS_DATA_00, 6, buffer);
      v.x = (((int16_t)buffer[1]) << 8) | buffer[0];
      v.y = (((int16_t)buffer[3]) << 8) | buffer[2];
      v.z = (((int16_t)buffer[5]) << 8) | buffer[4];
      return 1;
    }

    virtual MagDeviceType getType() const override
    {
      return MAG_AK8963;
    }

    void setSampleAveraging(uint8_t averaging) override
    {
    }

    void setSampleRate(uint8_t rate) override
    {
    }

    void setMode(uint8_t mode) override
    {
      // set AK8963 to Power Down
      writeSlave(AK8963_CNTL1, AK8963_PWR_DOWN);
      delay(100); // long wait between AK8963 mode changes

      // set AK8963 to FUSE ROM access
      writeSlave(AK8963_CNTL1, AK8963_FUSE_ROM);
      delay(100);

      /* get the magnetometer calibration */
      // read the AK8963 ASA registers and compute magnetometer scale factors
      uint8_t buffer[7];
      readSlave(AK8963_ASA, 3, buffer);
      scale[0] = ((((float)buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
      scale[1] = ((((float)buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
      scale[2] = ((((float)buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

      // set AK8963 to Power Down
      writeSlave(AK8963_CNTL1, AK8963_PWR_DOWN);
      delay(100);

      // set AK8963 to 16 bit resolution, 100 Hz update rate
      writeSlave(AK8963_CNTL1, AK8963_CNT_MEAS2);
      delay(100);

      // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
      readSlave(AK8963_HXL, 6, buffer);
    }

    void setGain(uint8_t gain) override
    {
    }

    bool testConnection() override
    {
      uint8_t buffer;
      if(readMaster(MPU9250_WHO_AM_I, 1, &buffer) != 1) return false;
      if(buffer != 0x71 && buffer != 0x73) return false;
      if(readSlave(AK8963_WHO_AM_I, 1, &buffer) != 1) return false;
      return buffer == 0x48;
    }

  private:
    int8_t writeMaster(uint8_t regAddr, uint8_t data)
    {
      int8_t res = _bus->writeByte(_masterAddr, regAddr, data);
      //delay(1);
      return res;
    }

    int8_t readMaster(uint8_t regAddr, uint8_t length, uint8_t *data)
    {
      return _bus->read(_masterAddr, regAddr, length, data);
    }

    int8_t readSlave(uint8_t regAddr, uint8_t length, uint8_t *data)
    {
      // set slave 0 to the AK8963 and set for read
      if(!writeMaster(MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80)) {
        return -1;
      }
      // set the register to the desired AK8963 sub address
      if(!writeMaster(MPU9250_I2C_SLV0_REG, regAddr)) {
        return -2;
      }
      // enable I2C and request the bytes
      if(!writeMaster(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | length)) {
        return -3;
      }

      // bit optimized burst version
      //uint8_t out[3] = { _addr | (uint8_t)0x80, regAddr, (uint8_t)MPU9250_I2C_SLV0_EN | length };
      //if(!_bus->write(_masterAddr, MPU9250_I2C_SLV0_ADDR, 3, out)) return -4;

      // takes some time for these registers to fill
      delay(1);

      // read the bytes off the MPU9250 EXT_SENS_DATA registers
      int8_t res = readMaster(MPU9250_EXT_SENS_DATA_00, length, data);
     // D("SR", regAddr, data[0]);

      return res;
    }

    int8_t writeSlave(uint8_t regAddr, uint8_t data)
    {
      // set slave 0 to the AK8963 and set for write
      if(!writeMaster(MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS)) {
        return -1;
      }
      // set the register to the desired AK8963 sub address 
      if(!writeMaster(MPU9250_I2C_SLV0_REG, regAddr)) {
        return -2;
      }
      // store the data for write
      if(!writeMaster(MPU9250_I2C_SLV0_DO, data)) {
        return -3;
      }
      // enable I2C and send 1 byte
      if(!writeMaster(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | 0x01)) {
        return -4;
      }

      //D("SW", regAddr, data);
      //delay(1);

      return true;

      // read the register and confirm
      uint8_t buffer;
      if(readSlave(regAddr, 1, &buffer) < 0) {
        return -5;
      }

      return buffer == data ? 1 : -6;
    }

    uint8_t _mode;
    float scale[3];
};

}

}

#endif
