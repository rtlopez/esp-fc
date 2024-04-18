#ifndef _ESPFC_DEVICE_BUSSPI_H_
#define _ESPFC_DEVICE_BUSSPI_H_

#include "BusDevice.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Device {

class BusSPI: public BusDevice
{
  public:
    BusSPI(ESPFC_SPI_0_DEV_T& spi);

    static const uint8_t  SPI_READ  = 0x80;
    static const uint8_t  SPI_WRITE = 0x7f;
    
    static const uint32_t SPI_SPEED_NORMAL = 1000000;
    static const uint32_t SPI_SPEED_FAST  = 16000000;

    BusType getType() const override;

    int begin(int8_t sck = -1, int8_t mosi = -1, int8_t miso = -1, int8_t ss = -1);

    int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override;

    int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override FAST_CODE_ATTR;

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override;

  private:
    void transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *in, uint8_t *out, uint32_t speed) FAST_CODE_ATTR;

    ESPFC_SPI_0_DEV_T& _dev;
};

}

}

#endif
