#ifndef _ESPFC_DEVICE_MAG_AK8963_H_
#define _ESPFC_DEVICE_MAG_AK8963_H_

#include "MagDevice.h"
#include "BusDevice.h"

#define AK8963_ADDRESS_FIRST      0x0C // this device only has one address
#define AK8963_ADDRESS_SECOND     0x0D // 0x0E and 0x0F also possible
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
#define AK8963_INIT_DELAY         20

namespace Espfc {

namespace Device {

class MagAK8963: public MagDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, AK8963_ADDRESS_FIRST) ? 1 : begin(bus, AK8963_ADDRESS_SECOND);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      // set AK8963 to Power Down
      _bus->writeByte(_addr, AK8963_CNTL1, AK8963_PWR_DOWN);
      delay(AK8963_INIT_DELAY); // long wait between AK8963 mode changes

      // set AK8963 to FUSE ROM access
      _bus->writeByte(_addr, AK8963_CNTL1, AK8963_FUSE_ROM);
      delay(AK8963_INIT_DELAY);

      /* get the magnetometer calibration */
      // read the AK8963 ASA registers and compute magnetometer scale factors
      _bus->read(_addr, AK8963_ASA, 3, buffer);
      // align CW90_FLIP (swap X Y)
      scale.y = ((((float)buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
      scale.x = ((((float)buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
      scale.z = ((((float)buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
      //scale *= 10.f; // convert to milli Gauss
      scale *= 0.01f; // convert to Gauss

      // set AK8963 to Power Down
      _bus->writeByte(_addr, AK8963_CNTL1, AK8963_PWR_DOWN);
      delay(AK8963_INIT_DELAY);

      // set AK8963 to 16 bit resolution, 100 Hz update rate
      _bus->writeByte(_addr, AK8963_CNTL1, AK8963_CNT_MEAS2);
      delay(AK8963_INIT_DELAY);

      // instruct master to get 7 bytes of data from the AK8963 at the sample rate
      _bus->read(_addr, AK8963_HXL, 7, buffer);

      return 1;
    }

    int readMag(VectorInt16& v) override
    {
      _bus->readFast(_addr, AK8963_HXL, 7, buffer);

      // align CW90_FLIP (swap X Y, invert Z)
      v.y =  ((((int16_t)buffer[1]) << 8) | buffer[0]);
      v.x =  ((((int16_t)buffer[3]) << 8) | buffer[2]);
      v.z = -((((int16_t)buffer[5]) << 8) | buffer[4]);
      return 1;
    }

    const VectorFloat convert(const VectorInt16& v) const override
    {
      return (VectorFloat)v * scale;
    }

    int getRate() const override
    {
      return 100;
    }

    virtual MagDeviceType getType() const override
    {
      return MAG_AK8963;
    }

    bool testConnection() override
    {
      uint8_t res = _bus->read(_addr, AK8963_WHO_AM_I, 1, buffer);
      //D("ak8963:whoami", _addr, buffer[0], res);
      return res == 1 && buffer[0] == 0x48;
    }

  private:
    uint8_t _mode;
    VectorFloat scale;
    uint8_t buffer[7];
};

}

}

#endif
