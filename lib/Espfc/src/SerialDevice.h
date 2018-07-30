#ifndef _ESPFC_SERIAL_DEVICE_H_
#define _ESPFC_SERIAL_DEVICE_H_

#include <Arduino.h>

#if defined(ESP32)
#include "soc/uart_struct.h"
#endif

#if defined(ESP8266)
  #define SERIAL_RXD_INV (1  <<  UCRXI) // bit 19 - invert rx
  #define SERIAL_TXD_INV (1  <<  UCTXI) // bit 22 - invert tx
#endif

#define SERIAL_UART_PARITY_NONE      0B00000000
#define SERIAL_UART_PARITY_EVEN      0B00000010
#define SERIAL_UART_PARITY_ODD       0B00000011

#define SERIAL_UART_NB_BIT_5         0B00000000
#define SERIAL_UART_NB_BIT_6         0B00000100
#define SERIAL_UART_NB_BIT_7         0B00001000
#define SERIAL_UART_NB_BIT_8         0B00001100

#define SERIAL_UART_NB_STOP_BIT_0    0B00000000
#define SERIAL_UART_NB_STOP_BIT_1    0B00010000
#define SERIAL_UART_NB_STOP_BIT_15   0B00100000
#define SERIAL_UART_NB_STOP_BIT_2    0B00110000

namespace Espfc {

enum SerialDeviceConfigParity {
  SERIAL_PARITY_NONE,
  SERIAL_PARITY_EVEN,
  SERIAL_PARITY_ODD
};

enum SerialDeviceConfigStopBits {
  SERIAL_STOP_BITS_0,
  SERIAL_STOP_BITS_1,
  SERIAL_STOP_BITS_15,
  SERIAL_STOP_BITS_2
};

class SerialDeviceConfig
{
  public:
    SerialDeviceConfig():
      baud(115200), rx_pin(-1), tx_pin(-1), inverted(false), data_bits(8), parity(SERIAL_PARITY_NONE), stop_bits(1)  {}
    uint32_t baud;
    int8_t rx_pin;
    int8_t tx_pin;
    bool inverted;
    int8_t data_bits;
    int8_t parity;
    int8_t stop_bits;
};

class SerialDevice: public Stream
{
  public:
    virtual void begin(const SerialDeviceConfig& conf) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
    virtual size_t write(uint8_t c) = 0;
    virtual size_t availableForWrite() = 0;
    virtual bool isSoft() const = 0;
    using Print::write;
};

}

#endif // _ESPFC_SERIAL_DEVICE_H_
