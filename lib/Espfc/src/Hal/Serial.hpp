#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Hal {

enum SerialDeviceConfigParity : uint8_t
{
  SDC_SERIAL_PARITY_NONE,
  SDC_SERIAL_PARITY_EVEN,
  SDC_SERIAL_PARITY_ODD,
};

enum SerialDeviceConfigStopBits : uint8_t
{
  SDC_SERIAL_STOP_BITS_0,
  SDC_SERIAL_STOP_BITS_1,
  SDC_SERIAL_STOP_BITS_15,
  SDC_SERIAL_STOP_BITS_2,
};

struct SerialDeviceConfig
{
  uint32_t baud = 115200;
  int8_t rx_pin = -1;
  int8_t tx_pin = -1;
  bool inverted = false;
  int8_t data_bits = 8;
  SerialDeviceConfigParity parity = SDC_SERIAL_PARITY_NONE;
  SerialDeviceConfigStopBits stop_bits = SDC_SERIAL_STOP_BITS_1;
};

class SerialUart
{
public:
  SerialUart(int index) : _index(index) {}
  void begin(const SerialDeviceConfig& conf);
  void updateBaudRate(int baud);
  int available();
  int read();
  size_t readMany(uint8_t* c, size_t l);
  int peek();
  void flush();
  size_t write(uint8_t c);
  size_t write(const uint8_t* c, size_t l);
  int availableForWrite();
  bool isTxFifoEmpty();
private:
  int _index;
};

SerialUart* getSerialUart(int index);

#if ARDUINO_USB_CDC_ON_BOOT || ARCH_RP2040

class SerialUsb
{
public:
  void begin(const SerialDeviceConfig& conf);
  void updateBaudRate(int baud);
  int available();
  int read();
  size_t readMany(uint8_t* c, size_t l);
  int peek();
  void flush();
  size_t write(uint8_t c);
  size_t write(const uint8_t* c, size_t l);
  int availableForWrite();
  bool isTxFifoEmpty();
};

SerialUsb* getSerialUsb();

#endif

} // namespace Espfc::Hal
