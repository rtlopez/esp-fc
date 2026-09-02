#if defined(ESP8266)

#include "Hal/Serial.hpp"
#include <Arduino.h>

namespace {

static constexpr uint32_t SERIAL_UART_PARITY_NONE = 0B00000000;
static constexpr uint32_t SERIAL_UART_PARITY_EVEN = 0B00000010;
static constexpr uint32_t SERIAL_UART_PARITY_ODD = 0B00000011;

static constexpr uint32_t SERIAL_UART_NB_BIT_5 = 0B00000000;
static constexpr uint32_t SERIAL_UART_NB_BIT_6 = 0B00000100;
static constexpr uint32_t SERIAL_UART_NB_BIT_7 = 0B00001000;
static constexpr uint32_t SERIAL_UART_NB_BIT_8 = 0B00001100;

static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_0 = 0B00000000;
static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_1 = 0B00010000;
static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_15 = 0B00100000;
static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_2 = 0B00110000;

static Espfc::Hal::SerialUart uart0(0);
static Espfc::Hal::SerialUart uart1(1);

} // namespace

static inline HardwareSerial& getPort(int index)
{
  switch (index)
  {
    case 1:
      return Serial1;
  }
  return Serial;
}

namespace Espfc::Hal {

SerialUart* getSerialUart(int index)
{
  switch (index)
  {
    case 0:
      return &uart0;
    case 1:
      return &uart1;
  }
  return nullptr;
}

static inline uint32_t targetSerialConfigFlags(const SerialDeviceConfig& conf)
{
  uint32_t sc = 0;
  // clang-format off
  switch(conf.data_bits)
  {
    case 8: sc |= UART_NB_BIT_8; break;
    case 7: sc |= UART_NB_BIT_7; break;
    case 6: sc |= UART_NB_BIT_6; break;
    case 5: sc |= UART_NB_BIT_5; break;
    default: sc |= UART_NB_BIT_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: sc |= UART_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  sc |= UART_PARITY_ODD;  break;
    default: sc |= UART_PARITY_NONE;  break;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  sc |= UART_NB_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: sc |= UART_NB_STOP_BIT_15; break;
    case SDC_SERIAL_STOP_BITS_1:  sc |= UART_NB_STOP_BIT_1;  break;
    default: break;
  }
  // clang-format on
  return sc;
}

static constexpr size_t targetSerialTxBufferSize()
{
  return 0xFF;
}

void SerialUart::begin(const SerialDeviceConfig& conf)
{
  uint32_t sc = targetSerialConfigFlags(conf);
  const bool isUart0 = _index == 0;
  getPort(_index).begin(conf.baud, (SerialConfig)sc, isUart0 ? SERIAL_FULL : SERIAL_TX_ONLY, isUart0 ? 1 : 2,
                        conf.inverted);
}

void SerialUart::updateBaudRate(int baud)
{
  getPort(_index).updateBaudRate(baud);
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
  return getPort(_index).readBytes((char*)c, l);
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

#if ARDUINO_USB_CDC_ON_BOOT

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
  return Serial.read((char*)c, l);
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

#endif

} // namespace Espfc::Hal

#endif
