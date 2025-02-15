#pragma once

#include "Esp.h"
#include "Debug_Espfc.h"
#include "Device/SerialDevice.h"

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

#define ESPFC_WIFI
#define ESPFC_ESPNOW

namespace Espfc {

template<typename T>
inline int targetSerialInit(T& dev, const SerialDeviceConfig& conf)
{
  uint32_t sc = 0x8000000;
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
    case SDC_SERIAL_PARITY_EVEN: sc |= SERIAL_UART_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  sc |= SERIAL_UART_PARITY_ODD;  break;
    default: sc |= SERIAL_UART_PARITY_NONE; break;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  sc |= SERIAL_UART_NB_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: sc |= SERIAL_UART_NB_STOP_BIT_15; break;
    case SDC_SERIAL_STOP_BITS_1:  sc |= SERIAL_UART_NB_STOP_BIT_1;  break;
    default: break;
  }
  if(dev) dev.end();
  dev.setTxBufferSize(SERIAL_TX_FIFO_SIZE);
  dev.begin(conf.baud, sc, conf.rx_pin, conf.tx_pin, conf.inverted);
  return 1;
}

template<typename T>
inline int targetSPIInit(T& dev, int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  dev.begin(sck, miso, mosi, ss);
  return 1;
}

template<typename T>
inline int targetI2CInit(T& dev, int8_t sda, int8_t scl, uint32_t speed)
{
  dev.begin(sda, scl, speed);
  dev.setTimeout(50);
  return 1;
}

inline uint32_t getBoardId0()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)mac;
}

inline uint32_t getBoardId1()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)(mac >> 32);
}

inline uint32_t getBoardId2()
{
  return 0;
}

inline void targetReset()
{
  ESP.restart();
  while(1) {}
}

inline uint32_t targetCpuFreq()
{
  return ESP.getCpuFreqMHz();
}

inline uint32_t targetFreeHeap()
{
  return ESP.getFreeHeap();
}

};
