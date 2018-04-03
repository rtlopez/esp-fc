#ifndef _ESPFC_DEVICE_BUSSPI_H_
#define _ESPFC_DEVICE_BUSSPI_H_

#include "BusDevice.h"
#include "SPI.h"

namespace Espfc {

namespace Device {

class BusSPI: public BusDevice
{
  public:
    int begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss)
    {
      SPI.begin(sck, miso, mosi, ss);
      return 0;
    }

    int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT) override
    {
      transfer(devAddr, regAddr, length, data, NULL);
      return length;
    }

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
    {
      transfer(devAddr, regAddr, length, NULL, data);
      return true;
    }

  private:
    void transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *out, uint8_t *in)
    {
      SPI.beginTransaction(SPISettings(8000000, LSBFIRST, SPI_MODE0));
      digitalWrite(devAddr, LOW);
      SPI.transfer(regAddr); // specify the starting register address
      SPI.transferBytes(out, in, length);
      /*for(uint8_t i = 0; i < length; i++)
      {
        uint8_t v = SPI.transfer(in ? in[i] : 0);
        if(out) out[i] = v; // write the data
      }*/
      digitalWrite(devAddr, HIGH);
      SPI.endTransaction();
    }
};

}

}

#endif
