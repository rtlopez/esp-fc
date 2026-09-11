#if defined(ESP32)

#include "Hal/BusSPI.hpp"

#include "Hal/Gpio.hpp"
#include <Arduino.h>
#include <SPI.h>

#if defined(ESP32C3)
#define NUM_SPI_PORTS 1
#else
#define NUM_SPI_PORTS 2
#endif

namespace {

static SPIClass SPI0(HSPI);
#if NUM_SPI_PORTS >= 2
#if defined(ESP32S2) || defined(ESP32S3)
static SPIClass SPI1(FSPI);
#else
static SPIClass SPI1(VSPI);
#endif
#endif

SPIClass& getSPI(size_t index)
{
  switch (index)
  {
    case 0:
      return SPI0;
#if NUM_SPI_PORTS >= 2
    case 1:
      return SPI1;
#endif
    default:
      return SPI0;
  }
}

Espfc::Hal::BusSPI _spi0(0);
#if NUM_SPI_PORTS >= 2
Espfc::Hal::BusSPI _spi1(1);
#endif

} // namespace

namespace Espfc::Hal {

BusSPI* getBusSPI(size_t index)
{
  switch (index)
  {
    case 0:
      return &_spi0;
#if NUM_SPI_PORTS >= 2
    case 1:
      return &_spi1;
#endif
    default:
      return nullptr;
  }
}

BusSPI::BusSPI(size_t index): _index(index) {}

BusType BusSPI::getType() const
{
  return BUS_SPI;
}

int BusSPI::begin(int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  if (sck == -1 || miso == -1 || mosi == -1) return 0;

  getSPI(_index).begin(sck, miso, mosi, ss);

  return 1;
}

int8_t BusSPI::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  transfer(devAddr, regAddr | SPI_READ, length, nullptr, data, SPI_SPEED_NORMAL);
  return length;
}

int8_t IRAM_ATTR BusSPI::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  transfer(devAddr, regAddr | SPI_READ, length, nullptr, data, SPI_SPEED_FAST);
  return length;
}

bool BusSPI::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  transfer(devAddr, regAddr & SPI_WRITE, length, data, nullptr, SPI_SPEED_NORMAL);
  return true;
}

void IRAM_ATTR BusSPI::transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* in, uint8_t* out,
                                uint32_t speed)
{
  auto& dev = getSPI(_index);
  dev.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
  Hal::Gpio::digitalWrite(devAddr, Hal::Gpio::Low);
  dev.transfer(regAddr);
  dev.transferBytes(in, out, length);
  Hal::Gpio::digitalWrite(devAddr, Hal::Gpio::High);
  dev.endTransaction();
}

} // namespace Espfc::Hal

#endif
