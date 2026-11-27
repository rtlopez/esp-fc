#if defined(UNIT_TEST)

#include "Hal/Serial.hpp"

namespace Hal {

static SerialUart uart0(0);

SerialUart* getSerialUart(int index)
{
  return index == 0 ? &uart0 : nullptr;
}

void SerialUart::begin(const SerialDeviceConfig& conf) {}

void SerialUart::updateBaudRate(int baud) {}

int SerialUart::available()
{
  return 0;
}

int SerialUart::read()
{
  return -1;
}

size_t SerialUart::readMany(uint8_t* c, size_t l)
{
  return 0;
}

int SerialUart::peek()
{
  return -1;
}

void SerialUart::flush() {}

size_t SerialUart::write(uint8_t c)
{
  return 1;
}

size_t SerialUart::write(const uint8_t* c, size_t l)
{
  return l;
}

int SerialUart::availableForWrite()
{
  return SERIAL_TX_BUF_LEN;
}

bool SerialUart::isTxFifoEmpty()
{
  return true;
}

} // namespace Hal

#endif
