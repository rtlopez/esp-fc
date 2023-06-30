#ifndef _ESPFC_SERIAL_DEVICE_ADAPTER_H_
#define _ESPFC_SERIAL_DEVICE_ADAPTER_H_

#include "Device/SerialDevice.h"
#ifdef ESPFC_SERIAL_SOFT_0_RX
#include "EspSoftSerial.h"
#endif
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
    virtual void begin(const SerialDeviceConfig& conf);
    virtual int available() { return _dev.available(); }
    virtual int read() { return _dev.read(); }
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

template<typename T>
void SerialDeviceAdapter<T>::begin(const SerialDeviceConfig& conf)
{
  targetSerialInit(_dev, conf);
}

// WiFiClient specializations
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
template<>
void SerialDeviceAdapter<WiFiClient>::begin(const SerialDeviceConfig& conf)
{
}

template<>
int SerialDeviceAdapter<WiFiClient>::availableForWrite()
{
  return SERIAL_TX_FIFO_SIZE;
}

template<>
bool SerialDeviceAdapter<WiFiClient>::isTxFifoEmpty()
{
  return true;
}
#endif

#ifdef ESPFC_SERIAL_SOFT_0_RX
template<>
void SerialDeviceAdapter<EspSoftSerial>::begin(const SerialDeviceConfig& conf)
{
  EspSoftSerialConfig ec;
  ec.baud = conf.baud;
  ec.rx_pin = conf.rx_pin;
  ec.inverted = conf.inverted;
  ec.data_bits = conf.data_bits;
  ec.parity_type = conf.parity;
  ec.stop_bits = conf.stop_bits;
  _dev.begin(ec);
}

template<>
bool SerialDeviceAdapter<EspSoftSerial>::isSoft() const
{
  return true;
}
#endif

}

}

#endif