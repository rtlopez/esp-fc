#pragma once

#define MPU6050_I2C_SLV0_ADDR     0x25
#define MPU6050_I2C_SLV0_REG      0x26
#define MPU6050_I2C_SLV0_DO       0x63
#define MPU6050_I2C_SLV0_CTRL     0x27
#define MPU6050_I2C_SLV0_EN       0x80
#define MPU6050_EXT_SENS_DATA_00  0x49

namespace Espfc {

namespace Device {

class BusSlave: public BusDevice, public BusAwareDevice
{
  public:
    BusSlave() {}

    int begin(BusDevice * dev, int addr)
    {
      setBus(dev, addr);

      return 1;
    }

    BusType getType() const override { return BUS_SLV; }

    virtual int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override
    {
      // set slave 0 to the AK8963 and set for read
      if(!writeMaster(MPU6050_I2C_SLV0_ADDR, devAddr | 0x80)) {
        return 0;
      }
      // set the register to the desired AK8963 sub address
      if(!writeMaster(MPU6050_I2C_SLV0_REG, regAddr)) {
        return 0;
      }
      // enable I2C and request the bytes
      if(!writeMaster(MPU6050_I2C_SLV0_CTRL, MPU6050_I2C_SLV0_EN | length)) {
        return 0;
      }

      // takes some time for these registers to fill
      delay(1);

      // read the bytes off the EXT_SENS_DATA registers
      int8_t res = readMaster(MPU6050_EXT_SENS_DATA_00, length, data);

      return res;
    }

    // readFast() ignores devAddr and regAddr args and read ext sensor data reg from master
    virtual int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override
    {
      return _bus->readFast(_addr, MPU6050_EXT_SENS_DATA_00, length, data);
    }

    // writes only one byte, length is ignored
    virtual bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override
    {
      // set slave 0 to the AK8963 and set for write
      if(!writeMaster(MPU6050_I2C_SLV0_ADDR, devAddr)) {
        return false;
      }
      // set the register to the desired AK8963 sub address 
      if(!writeMaster(MPU6050_I2C_SLV0_REG, regAddr)) {
        return false;
      }
      // store the data for write
      if(!writeMaster(MPU6050_I2C_SLV0_DO, *data)) {
        return false;
      }
      // enable I2C and send 1 byte
      if(!writeMaster(MPU6050_I2C_SLV0_CTRL, MPU6050_I2C_SLV0_EN | 0x01)) {
        return false;
      }

      return true;
    }

    int8_t writeMaster(uint8_t regAddr, uint8_t data)
    {
      int8_t res = _bus->write(_addr, regAddr, 1, &data);
      delay(10);
      return res;
    }

    int8_t readMaster(uint8_t regAddr, uint8_t length, uint8_t *data)
    {
      return _bus->read(_addr, regAddr, length, data);
    }
};

}

}