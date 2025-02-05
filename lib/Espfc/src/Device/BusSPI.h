#ifndef _ESPFC_DEVICE_BUSSPI_H_
#define _ESPFC_DEVICE_BUSSPI_H_

#include "BusDevice.h"

namespace Espfc {

namespace Device {

class BusSPI: public BusDevice
{
  public:
    BusSPI(ESPFC_SPI_0_DEV_T& spi);

    static constexpr uint8_t  SPI_READ  = 0x80;
    static constexpr uint8_t  SPI_WRITE = 0x7f;
    
    static constexpr uint32_t SPI_SPEED_NORMAL = 1000000;
    static constexpr uint32_t SPI_SPEED_FAST  = 16000000;

    BusType getType() const override;

    int begin(int8_t sck = -1, int8_t mosi = -1, int8_t miso = -1, int8_t ss = -1);

    int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override;

    int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) override;

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override;

  private:
    void transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *in, uint8_t *out, uint32_t speed);

    ESPFC_SPI_0_DEV_T& _dev;
};

}

}

#endif
