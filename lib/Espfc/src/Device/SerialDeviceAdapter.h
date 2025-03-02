#ifndef _ESPFC_SERIAL_DEVICE_ADAPTER_H_
#define _ESPFC_SERIAL_DEVICE_ADAPTER_H_

#include "Device/SerialDevice.h"
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#include <WiFiClient.h>
#endif

namespace Espfc {

namespace Device {

template<typename T>
class SerialDeviceAdapter: public SerialDevice
{
  public:
    SerialDeviceAdapter(T& dev): _dev(dev) {}
    void begin(const SerialDeviceConfig& conf) override { targetSerialInit(_dev, conf); }
    void updateBaudRate(int baud) override { _dev.updateBaudRate(baud); };
    int available() override { return _dev.available(); }
    int read() override { return _dev.read(); }
    size_t readMany(uint8_t * c, size_t l) override {
#if defined(ARCH_RP2040)
      size_t count = std::min(l, (size_t)available());
      for(size_t i = 0; i < count; i++)
      {
        c[i] = read();
      }
      return count;
#else
      return _dev.read(c, l);
#endif
    }
    int peek() override { return _dev.peek(); }
    void flush() override { _dev.flush(); }
    size_t write(uint8_t c) override { return _dev.write(c); }
    size_t write(const uint8_t * c, size_t l) override { return _dev.write(c, l); }
    bool isSoft() const override { return false; };
    int availableForWrite() override { return _dev.availableForWrite(); }
    bool isTxFifoEmpty() override { return _dev.availableForWrite() >= SERIAL_TX_FIFO_SIZE; }
    operator bool() const override { return (bool)_dev; }
  private:
    T& _dev;
};

// WiFiClient specializations
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
template<>
inline void SerialDeviceAdapter<WiFiClient>::begin(const SerialDeviceConfig& conf)
{
}

template<>
inline int SerialDeviceAdapter<WiFiClient>::availableForWrite()
{
  return SERIAL_TX_FIFO_SIZE;
}

template<>
inline bool SerialDeviceAdapter<WiFiClient>::isTxFifoEmpty()
{
  return true;
}

template<>
inline void SerialDeviceAdapter<WiFiClient>::updateBaudRate(int baud) {}

#endif

#if defined(ESP32C3) || defined(ESP32S3)
template<>
inline void SerialDeviceAdapter<HWCDC>::updateBaudRate(int baud) {}
#endif

#if defined(ESP32S2)
template<>
inline void SerialDeviceAdapter<USBCDC>::updateBaudRate(int baud) {}
#endif

#if defined(ARCH_RP2040)
template<>
inline void SerialDeviceAdapter<SerialUART>::updateBaudRate(int baud)
{
  _dev.begin(baud);
}

template<>
inline void SerialDeviceAdapter<SerialUSB>::updateBaudRate(int baud) {}
#endif

}

}

#endif