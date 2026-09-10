#if defined(ARCH_RP2040)

#include "Hal/BusSPI.hpp"

#include "Hal/Gpio.hpp"
#include <Arduino.h>
#include <SPI.h>

namespace {

// index 0 uses SPI1, because default target pins (sck 14, mosi 15, miso 12) belong to the spi1 peripheral
SPIClassRP2040& getSPI(size_t index) {
  switch (index) {
    case 0:
      return SPI1;
    case 1:
      return SPI;
    default:
      return SPI1;
  }
}

Espfc::Hal::BusSPI _spi0(0);
Espfc::Hal::BusSPI _spi1(1);

}

namespace Espfc::Hal {

BusSPI* getBusSPI(size_t index)
{
  switch (index) {
    case 0:
      return &_spi0;
    case 1:
      return &_spi1;
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
  (void)ss;

  if (sck == -1 || miso == -1 || mosi == -1) return 0;

  auto& dev = getSPI(_index);
  dev.setSCK(sck);
  dev.setRX(miso);
  dev.setTX(mosi);
  dev.begin();

  return 1;
}

int8_t BusSPI::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  transfer(devAddr, regAddr | SPI_READ, length, nullptr, data, SPI_SPEED_NORMAL);
  return length;
}

int8_t BusSPI::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  transfer(devAddr, regAddr | SPI_READ, length, nullptr, data, SPI_SPEED_FAST);
  return length;
}

bool BusSPI::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  transfer(devAddr, regAddr & SPI_WRITE, length, data, nullptr, SPI_SPEED_NORMAL);
  return true;
}

void BusSPI::transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* in, uint8_t* out,
                                     uint32_t speed)
{
  auto& dev = getSPI(_index);

  dev.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
  Hal::Gpio::digitalWrite(devAddr, Hal::Gpio::Low);
  dev.transfer(regAddr);
  dev.transfer(in, out, length);
  Hal::Gpio::digitalWrite(devAddr, Hal::Gpio::High);
  dev.endTransaction();
}

} // namespace Espfc::Hal

#endif
