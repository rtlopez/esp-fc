#if defined(ESP32)

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

#if ARDUINO_USB_CDC_ON_BOOT // Serial used for USB CDC
static Espfc::Hal::SerialUsb usb;
#endif

static Espfc::Hal::SerialUart uart0(0);

#if SOC_UART_NUM > 1
static Espfc::Hal::SerialUart uart1(1);
#endif

#if SOC_UART_NUM > 2
static Espfc::Hal::SerialUart uart2(2);
#endif

} // namespace

static inline HardwareSerial& getPort(int index)
{
  switch (index)
  {
#if SOC_UART_NUM > 1
    case 1:
      return Serial1;
#endif
#if SOC_UART_NUM > 2
    case 2:
      return Serial2;
#endif
  }
#if ARDUINO_USB_CDC_ON_BOOT // Serial used for USB CDC
  return Serial0;
#else
  return Serial;
#endif
}

namespace Espfc::Hal {

SerialUart* getSerialUart(int index)
{
  switch (index)
  {
    case 0:
      return &uart0;
#if SOC_UART_NUM > 1
    case 1:
      return &uart1;
#endif
#if SOC_UART_NUM > 2
    case 2:
      return &uart2;
#endif
  }
  return nullptr;
}

static inline uint32_t targetSerialConfigFlags(const Hal::SerialDeviceConfig& conf)
{
  uint32_t sc = 0x8000000;
  // clang-format off
  switch(conf.data_bits)
  {
    case 8: sc |= SERIAL_UART_NB_BIT_8; break;
    case 7: sc |= SERIAL_UART_NB_BIT_7; break;
    case 6: sc |= SERIAL_UART_NB_BIT_6; break;
    case 5: sc |= SERIAL_UART_NB_BIT_5; break;
    default: sc |= SERIAL_UART_NB_BIT_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: sc |= SERIAL_UART_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  sc |= SERIAL_UART_PARITY_ODD;  break;
    default: sc |= SERIAL_UART_PARITY_NONE; break;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  sc |= SERIAL_UART_NB_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: sc |= SERIAL_UART_NB_STOP_BIT_15; break;
    case SDC_SERIAL_STOP_BITS_1:  sc |= SERIAL_UART_NB_STOP_BIT_1;  break;
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
  getPort(_index).end();
  getPort(_index).setTxBufferSize(targetSerialTxBufferSize());
  getPort(_index).begin(conf.baud, sc, conf.rx_pin, conf.tx_pin, conf.inverted);
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
