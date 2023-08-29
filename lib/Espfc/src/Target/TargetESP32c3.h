#pragma once

#include "Esp.h"
#include "Debug_Espfc.h"
//free pins left: IO15 IO16 IO17
#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 3 // ppm

#define ESPFC_OUTPUT_COUNT 4
#define ESPFC_OUTPUT_0 2
#define ESPFC_OUTPUT_1 4
#define ESPFC_OUTPUT_2 18
#define ESPFC_OUTPUT_3 19

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_DEV Serial
#define ESPFC_SERIAL_0_DEV_T HardwareSerial
#define ESPFC_SERIAL_0_TX 21
#define ESPFC_SERIAL_0_RX 20
#define ESPFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_1
#define ESPFC_SERIAL_1_DEV Serial1
#define ESPFC_SERIAL_1_DEV_T HardwareSerial
#define ESPFC_SERIAL_1_TX -1
#define ESPFC_SERIAL_1_RX -1
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_REMAP_PINS
#define ESPFC_SERIAL_DEBUG_PORT SERIAL_UART_0
#define SERIAL_TX_FIFO_SIZE 0xFF

#define ESPFC_SPI_0
#define ESPFC_SPI_0_DEV SPI1
#define ESPFC_SPI_0_SCK 6
#define ESPFC_SPI_0_MOSI 14
#define ESPFC_SPI_0_MISO 7

#define ESPFC_SPI_CS_GYRO 10
#define ESPFC_SPI_CS_BARO 12

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 9
#define ESPFC_I2C_0_SDA 8
//#define ESPFC_I2C_0_SOFT

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN 0

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 1

#define ESPFC_ADC_1
#define ESPFC_ADC_1_PIN 5

#define ESPFC_ADC_SCALE (3.3f / 4096)

#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_DYNAMIC_FILTER)

#define ESPFC_GUARD 0

#define ESPFC_GYRO_I2C_RATE_MAX 2000
#define ESPFC_GYRO_SPI_RATE_MAX 8000

//#define ESPFC_LOGGER_FS // doesn't compile on ESP32
#define ESPFC_DSP

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
