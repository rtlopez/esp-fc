#ifndef _ESPFC_DEVICE_SERIAL_DEVICE_H_
#define _ESPFC_DEVICE_SERIAL_DEVICE_H_

#if defined(ESP8266)
#define USE_SOFT_SERIAL
#endif

#if defined(ESP32)
#define SERIAL_TX_FIFO_SIZE 0x7f
#endif

#if defined(ESP8266)
  #define SERIAL_RXD_INV (1  <<  UCRXI) // bit 19 - invert rx
  #define SERIAL_TXD_INV (1  <<  UCTXI) // bit 22 - invert tx
  #define SERIAL_TX_FIFO_SIZE 0x80
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

enum SerialSpeed {
  SERIAL_SPEED_NONE    =       0,
  SERIAL_SPEED_9600    =    9600,
  SERIAL_SPEED_19200   =   19200,
  SERIAL_SPEED_38400   =   38400,
  SERIAL_SPEED_57600   =   57600,
  SERIAL_SPEED_115200  =  115200,
  SERIAL_SPEED_230400  =  230400,
  SERIAL_SPEED_250000  =  250000,
  SERIAL_SPEED_400000  =  400000,
  SERIAL_SPEED_460800  =  460800,
  SERIAL_SPEED_500000  =  500000,
  SERIAL_SPEED_921600  =  921600,
  SERIAL_SPEED_1000000 = 1000000,
  SERIAL_SPEED_1500000 = 1500000,
  SERIAL_SPEED_2000000 = 2000000,
  SERIAL_SPEED_2470000 = 2470000,
};

enum SerialPort {
  SERIAL_UART_0,
  SERIAL_UART_1,
#if defined(ESP32)
  SERIAL_UART_2,
  SERIAL_SOFT_0, // wifi
#elif defined(USE_SOFT_SERIAL)
  SERIAL_SOFT_0,
#endif
  SERIAL_UART_COUNT
};

enum SerialPortId {
  SERIAL_ID_NONE = -1,
  SERIAL_ID_UART_1 = 0,
  SERIAL_ID_UART_2,
  SERIAL_ID_UART_3,
  SERIAL_ID_UART_4,
  SERIAL_ID_UART_5,
  SERIAL_ID_UART_6,
  SERIAL_ID_UART_7,
  SERIAL_ID_UART_8,
  SERIAL_ID_LPUART_1,
  SERIAL_ID_USB_VCP = 20,
  SERIAL_ID_SOFTSERIAL_1 = 30,
  SERIAL_ID_SOFTSERIAL_2,
  SERIAL_ID_MAX = SERIAL_ID_SOFTSERIAL_2,
};

enum SerialFunction {
  SERIAL_FUNCTION_NONE                = 0,
  SERIAL_FUNCTION_MSP                 = (1 << 0),  // 1
  SERIAL_FUNCTION_GPS                 = (1 << 1),  // 2
  SERIAL_FUNCTION_TELEMETRY_FRSKY     = (1 << 2),  // 4
  SERIAL_FUNCTION_TELEMETRY_HOTT      = (1 << 3),  // 8
  SERIAL_FUNCTION_TELEMETRY_LTM       = (1 << 4),  // 16
  SERIAL_FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
  SERIAL_FUNCTION_RX_SERIAL           = (1 << 6),  // 64
  SERIAL_FUNCTION_BLACKBOX            = (1 << 7),  // 128
  SERIAL_FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
  SERIAL_FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
  SERIAL_FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
  SERIAL_FUNCTION_TELEMETRY_IBUS      = (1 << 12), // 4096
  SERIAL_FUNCTION_VTX_TRAMP           = (1 << 13), // 8192
  SERIAL_FUNCTION_RCDEVICE            = (1 << 14), // 16384
  SERIAL_FUNCTION_LIDAR_TF            = (1 << 15), // 32768
  SERIAL_FUNCTION_FRSKY_OSD           = (1 << 16), // 65536
};

enum SerialRXProvider {
  SERIALRX_SPEKTRUM1024 = 0,
  SERIALRX_SPEKTRUM2048 = 1,
  SERIALRX_SBUS = 2,
  SERIALRX_SUMD = 3,
  SERIALRX_SUMH = 4,
  SERIALRX_XBUS_MODE_B = 5,
  SERIALRX_XBUS_MODE_B_RJ01 = 6,
  SERIALRX_IBUS = 7,
  SERIALRX_JETIEXBUS = 8,
  SERIALRX_CRSF = 9,
  SERIALRX_SRXL = 10,
  SERIALRX_TARGET_CUSTOM = 11,
  SERIALRX_FPORT = 12,
};

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

namespace Device {

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
    virtual bool isTxFifoEmpty() = 0;
    virtual bool isSoft() const = 0;
    using Print::write;
};

}

}

#endif // _ESPFC_SERIAL_DEVICE_H_
