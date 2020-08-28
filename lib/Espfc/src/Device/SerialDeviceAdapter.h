#ifndef _ESPFC_SERIAL_DEVICE_ADAPTER_H_
#define _ESPFC_SERIAL_DEVICE_ADAPTER_H_

#include "Device/SerialDevice.h"
#include "EspSoftSerial.h"
#include <WiFiClient.h>

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
    virtual bool isSoft() const { return false; };
    virtual size_t availableForWrite() { return _dev.availableForWrite(); }
    virtual bool isTxFifoEmpty() { return _dev.availableForWrite() >= SERIAL_TX_FIFO_SIZE; }
  private:
    T& _dev;
};

template<typename T>
void SerialDeviceAdapter<T>::begin(const SerialDeviceConfig& conf)
{
  uint32_t sc = 0;
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
    case SERIAL_PARITY_EVEN: sc |= SERIAL_UART_PARITY_EVEN; break;
    case SERIAL_PARITY_ODD:  sc |= SERIAL_UART_PARITY_ODD;  break;
    default: break;
  }
  switch(conf.stop_bits)
  {
    case SERIAL_STOP_BITS_2:  sc |= SERIAL_UART_NB_STOP_BIT_2;  break;
    case SERIAL_STOP_BITS_15: sc |= SERIAL_UART_NB_STOP_BIT_15; break;
    case SERIAL_STOP_BITS_1:  sc |= SERIAL_UART_NB_STOP_BIT_1;  break;
    default: break;
  }

#if defined(ESP8266)

  if(conf.inverted)
  {
    sc |= (SERIAL_RXD_INV | SERIAL_TXD_INV);
  }
  _dev.begin(conf.baud, (SerialConfig)sc);

#elif defined(ESP32)

  sc |= 0x8000000;
  _dev.begin(conf.baud, sc, conf.rx_pin, conf.tx_pin, conf.inverted);

#else

  #error "Unsupported platform"

#endif
}

// WiFiClient specializations
template<>
void SerialDeviceAdapter<WiFiClient>::begin(const SerialDeviceConfig& conf)
{
}

template<>
size_t SerialDeviceAdapter<WiFiClient>::availableForWrite()
{
  return SERIAL_TX_FIFO_SIZE;
}

template<>
bool SerialDeviceAdapter<WiFiClient>::isTxFifoEmpty()
{
  return true;
}

// EspSofSerial specialization
#if defined(ESP32)
// not applicable
#elif defined(USE_SOFT_SERIAL)

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
bool SerialDeviceAdapter<EspSoftSerial>::isSoft() const { return true; }

#endif

}

}

#endif