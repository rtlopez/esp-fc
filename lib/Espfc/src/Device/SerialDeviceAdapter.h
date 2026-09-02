#ifndef _ESPFC_SERIAL_DEVICE_ADAPTER_H_
#define _ESPFC_SERIAL_DEVICE_ADAPTER_H_

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#include <WiFiClient.h>
#endif
#include "Hal/Serial.hpp"
#include "Stream/ReadWritable.hpp"
namespace Espfc {

namespace Device {

template<typename T>
class SerialDeviceAdapter: public Stream::ReadWritable
{
  public:
    SerialDeviceAdapter(T& dev): _dev(dev) {}
    void begin(const Hal::SerialDeviceConfig& conf) override { _dev.begin(conf); }
    void updateBaudRate(int baud) override { _dev.updateBaudRate(baud); };

    int available() override { return _dev.available(); }
    int read() override { return _dev.read(); }
    size_t readMany(uint8_t * c, size_t l) override { return _dev.readMany(c, l); }
    int peek() override { return _dev.peek(); }
    
    int availableForWrite() override { return _dev.availableForWrite(); }
    size_t write(uint8_t c) override { return _dev.write(c); }
    size_t write(const uint8_t * c, size_t l) override { return _dev.write(c, l); }
    void flush() override { _dev.flush(); }
    bool isTxFifoEmpty() override { return _dev.isTxFifoEmpty(); }
  private:
    T& _dev;
};

// WiFiClient specializations
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
template<>
inline void SerialDeviceAdapter<WiFiClient>::begin(const Hal::SerialDeviceConfig& conf)
{
}

template<>
inline int SerialDeviceAdapter<WiFiClient>::availableForWrite()
{
  return static_cast<int>(targetSerialTxBufferSize());
}

template<>
inline bool SerialDeviceAdapter<WiFiClient>::isTxFifoEmpty()
{
  return true;
}

template<>
inline void SerialDeviceAdapter<WiFiClient>::updateBaudRate(int baud) {}

template<>
inline size_t SerialDeviceAdapter<WiFiClient>::readMany(uint8_t * c, size_t l)
{
  return _dev.read(c, l);
}

#endif

// #if defined(ESP32C3) || defined(ESP32S3)
// template<>
// inline void SerialDeviceAdapter<HWCDC>::updateBaudRate(int baud) {}
// #endif

// #if defined(ESP32S2)
// template<>
// inline void SerialDeviceAdapter<USBCDC>::updateBaudRate(int baud) {}
// #endif

// #if defined(ARCH_RP2040)
// template<>
// inline void SerialDeviceAdapter<SerialUART>::updateBaudRate(int baud)
// {
//   _dev.begin(baud);
// }

// template<>
// inline void SerialDeviceAdapter<SerialUSB>::updateBaudRate(int baud) {}
// #endif

}

}

#endif