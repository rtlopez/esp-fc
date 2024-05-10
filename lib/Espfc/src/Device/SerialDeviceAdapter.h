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
    virtual void begin(const SerialDeviceConfig& conf) { targetSerialInit(_dev, conf); }
    virtual int available() { return _dev.available(); }
    virtual int read() { return _dev.read(); }
    virtual size_t readMany(uint8_t * c, size_t l) {
#ifdef TARGET_RP2040
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
    virtual int peek() { return _dev.peek(); }
    virtual void flush() { _dev.flush(); }
    virtual size_t write(uint8_t c) { return _dev.write(c); }
    virtual size_t write(const uint8_t * c, size_t l) { return _dev.write(c, l); }
    virtual bool isSoft() const { return false; };
    virtual int availableForWrite() { return _dev.availableForWrite(); }
    virtual bool isTxFifoEmpty() { return _dev.availableForWrite() >= SERIAL_TX_FIFO_SIZE; }
    virtual operator bool() const { return (bool)_dev; }
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
#endif

}

}

#endif