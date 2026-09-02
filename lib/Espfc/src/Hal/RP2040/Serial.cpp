#if defined(ARCH_RP2040)

#include "Hal/Serial.hpp"
#include <Arduino.h>
#include <algorithm>

namespace {

static Espfc::Hal::SerialUsb usb;
static Espfc::Hal::SerialUart _uart0(0);
static Espfc::Hal::SerialUart _uart1(1);

} // namespace

static inline SerialUART& getPort(int index)
{
  switch (index)
  {
    case 1:
      return Serial2;
  }
  return Serial1;
}

namespace Espfc::Hal {

SerialUart* getSerialUart(int index)
{
  switch (index)
  {
    case 0:
      return &_uart0;
    case 1:
      return &_uart1;
  }
  return nullptr;
}

static inline uint16_t targetSerialConfigFlags(const Hal::SerialDeviceConfig& conf)
{
  uint16_t flags = 0;
  // clang-format off
  switch(conf.data_bits)
  {
    case 8:  flags |= SERIAL_DATA_8; break;
    case 7:  flags |= SERIAL_DATA_7; break;
    case 6:  flags |= SERIAL_DATA_6; break;
    case 5:  flags |= SERIAL_DATA_5; break;
    default: flags |= SERIAL_DATA_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: flags |= SERIAL_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  flags |= SERIAL_PARITY_ODD;  break;
    default: flags |= SERIAL_PARITY_NONE;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  flags |= SERIAL_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: flags |= SERIAL_STOP_BIT_1_5; break;
    case SDC_SERIAL_STOP_BITS_1:  flags |= SERIAL_STOP_BIT_1;  break;
    default: break;
  }
  // clang-format on
  return flags;
}

static constexpr size_t targetSerialTxBufferSize()
{
  return 256u;
}

void SerialUart::begin(const SerialDeviceConfig& conf)
{
  uint16_t sc = targetSerialConfigFlags(conf);
  auto& p = getPort(_index);
  p.setFIFOSize(targetSerialTxBufferSize());
  p.setPinout(conf.tx_pin, conf.rx_pin);
  if (conf.inverted)
  {
    // gpio_set_inover(conf.rx_pin, GPIO_OVERRIDE_INVERT);
    // gpio_set_outover(conf.tx_pin, GPIO_OVERRIDE_INVERT);
    p.setInvertRX();
    p.setInvertTX();
  }
  p.begin(conf.baud, sc);
}

void SerialUart::updateBaudRate(int baud)
{
  // getPort(_index).updateBaudRate(baud);
}

int SerialUart::available()
{
  return getPort(_index).available();
}

int SerialUart::read()
{
  return getPort(_index).read();
}

size_t SerialUart::readMany(uint8_t* c, size_t l)
{
  auto& p = getPort(_index);
  auto len = std::min<size_t>(l, p.available());
  for (size_t i = 0; i < len; i++)
  {
    c[i] = p.read();
  }
  return len;
}

int SerialUart::peek()
{
  return getPort(_index).peek();
}

void SerialUart::flush() {}

size_t SerialUart::write(uint8_t c)
{
  return getPort(_index).write(c);
}

size_t SerialUart::write(const uint8_t* c, size_t l)
{
  return getPort(_index).write(c, l);
}

int SerialUart::availableForWrite()
{
  return getPort(_index).availableForWrite();
}

bool SerialUart::isTxFifoEmpty()
{
  return getPort(_index).availableForWrite() >= 0xff;
}

SerialUsb* getSerialUsb()
{
  return &usb;
}

void SerialUsb::begin(const SerialDeviceConfig& conf)
{
  Serial.begin(conf.baud);
}

void SerialUsb::updateBaudRate(int baud)
{
  // noop
}

int SerialUsb::available()
{
  return Serial.available();
}

int SerialUsb::read()
{
  return Serial.read();
}

size_t SerialUsb::readMany(uint8_t* c, size_t l)
{
  auto& p = Serial;
  auto len = std::min<size_t>(l, p.available());
  for (size_t i = 0; i < len; i++)
  {
    c[i] = p.read();
  }
  return len;
}

int SerialUsb::peek()
{
  return Serial.peek();
}

void SerialUsb::flush() {}

size_t SerialUsb::write(uint8_t c)
{
  return Serial.write(c);
}

size_t SerialUsb::write(const uint8_t* c, size_t l)
{
  return Serial.write(c, l);
}

int SerialUsb::availableForWrite()
{
  return Serial.availableForWrite();
}

bool SerialUsb::isTxFifoEmpty()
{
  return true;
}

} // namespace Espfc::Hal

#endif
