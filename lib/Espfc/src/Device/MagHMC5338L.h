#ifndef _ESPFC_DEVICE_MAG_HMC5338L_H_
#define _ESPFC_DEVICE_MAG_HMC5338L_H_

#include "MagDevice.h"
#include "BusDevice.h"

#define HMC5883L_ADDRESS            0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    0x1E

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0


namespace Espfc {

namespace Device {

class MagHMC5338L: public MagDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      return begin(bus, HMC5883L_DEFAULT_ADDRESS, 0);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      return begin(bus, addr, 0);
    }

    int begin(BusDevice * bus, uint8_t addr, uint8_t masterAddr) override
    {
      setBus(bus, addr, masterAddr);

      return testConnection();

      setMode(HMC5883L_MODE_CONTINUOUS);
      setSampleAveraging(HMC5883L_AVERAGING_1);
      setSampleRate(HMC5883L_RATE_75);
      setGain(HMC5883L_GAIN_1090);
    }

    int readMag(VectorInt16& v) override
    {
      uint8_t buffer[6];
      _bus->read(_addr, HMC5883L_RA_DATAX_H, 6, buffer);
      if (_mode == HMC5883L_MODE_SINGLE)
      {
        _bus->writeByte(_addr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
      }
      v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
      v.z = (((int16_t)buffer[2]) << 8) | buffer[3];
      v.y = (((int16_t)buffer[4]) << 8) | buffer[5];
      return 1;
    }

    const VectorFloat convert(const VectorInt16& v) const override
    {
      const float scale = 1.f / 1090.f;
      return VectorFloat(v) * scale;
    }

    int getRate() const override
    {
      return 75;
    }

    virtual MagDeviceType getType() const override
    {
      return MAG_HMC5883;
    }

    void setSampleAveraging(uint8_t averaging)
    {
      _bus->writeBits(_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
    }

    void setSampleRate(uint8_t rate)
    {
      _bus->writeBits(_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
    }

    void setMode(uint8_t mode)
    {
      _bus->writeByte(_addr, HMC5883L_RA_MODE, mode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
      _mode = mode; // track to tell if we have to clear bit 7 after a read
    }

    void setGain(uint8_t gain)
    {
      _bus->writeByte(_addr, HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
    }

    bool testConnection() override
    {
      uint8_t buffer[3];
      if(_bus->read(_addr, HMC5883L_RA_ID_A, 3, buffer) == 3)
      {
        return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
      }
      return false;
    }

  private:
    uint8_t _mode;
};

}

}

#endif
