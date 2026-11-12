#ifndef _ESPFC_SERIAL_DEVICE_ADAPTER_H_
#define _ESPFC_SERIAL_DEVICE_ADAPTER_H_

#include "Hal/Serial.hpp"
#include "Stream/ReadWritable.hpp"
namespace Espfc::Device {

template<typename T>
class SerialDeviceAdapter : public Stream::ReadWritable
{
public:
  SerialDeviceAdapter(T& dev): _dev(dev) {}
  void begin(const Hal::SerialDeviceConfig& conf) override
  {
    _dev.begin(conf);
  }

  void updateBaudRate(int baud) override
  {
    _dev.updateBaudRate(baud);
  };

  int available() override
  {
    return _dev.available();
  }

  int read() override
  {
    return _dev.read();
  }

  size_t readMany(uint8_t* c, size_t l) override
  {
    return _dev.readMany(c, l);
  }

  int peek() override
  {
    return _dev.peek();
  }

  int availableForWrite() override
  {
    return _dev.availableForWrite();
  }

  size_t write(uint8_t c) override
  {
    return _dev.write(c);
  }

  size_t write(const uint8_t* c, size_t l) override
  {
    return _dev.write(c, l);
  }

  void flush() override
  {
    _dev.flush();
  }

  bool isTxFifoEmpty() override
  {
    return _dev.isTxFifoEmpty();
  }

private:
  T& _dev;
};

} // namespace Espfc::Device

#endif