#include "Target/Target.h"

#if defined(ESPFC_SPI_0)

#include "BusSPI.h"
#include <Arduino.h>

namespace Espfc {

namespace Device {

BusSPI::BusSPI(ESPFC_SPI_0_DEV_T& spi): _dev(spi) {}

BusType BusSPI::getType() const { return BUS_SPI; }

int BusSPI::begin(int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
    if(sck == -1 || miso == -1 || mosi == -1) return 0;

    targetSPIInit(_dev, sck, mosi, miso, ss);

    return 1;
}

int8_t BusSPI::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    //D("spi:r", regAddr, length);
    transfer(devAddr, regAddr | SPI_READ, length, NULL, data, SPI_SPEED_NORMAL);
    return length;
}

int8_t FAST_CODE_ATTR BusSPI::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    //D("spi:r", regAddr, length);
    transfer(devAddr, regAddr | SPI_READ, length, NULL, data, SPI_SPEED_FAST);
    return length;
}

bool BusSPI::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
    //D("spi:w", regAddr, length, *data);
    transfer(devAddr, regAddr & SPI_WRITE, length, data, NULL, SPI_SPEED_NORMAL);
    return true;
}

void FAST_CODE_ATTR BusSPI::transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *in, uint8_t *out, uint32_t speed)
{
    _dev.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
    Hal::Gpio::digitalWrite(devAddr, LOW);
#if defined (ARCH_RP2040)
    _dev.transfer(regAddr);
    _dev.transfer(in, out, length);
#else
    _dev.transfer(regAddr);
    _dev.transferBytes(in, out, length);
#endif
    Hal::Gpio::digitalWrite(devAddr, HIGH);
    _dev.endTransaction();
}

}

}

#endif