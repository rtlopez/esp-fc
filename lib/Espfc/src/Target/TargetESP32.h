#pragma once

#include "Esp.h"

#define ESPFC_SPI_0
#define ESPFC_SPI_0_SCK 18
#define ESPFC_SPI_0_MOSI 23
#define ESPFC_SPI_0_MISO 19

#define ESPFC_SPI_CS_GYRO 5
#define ESPFC_SPI_CS_BARO 15

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 22
#define ESPFC_I2C_0_SDA 21

#define ESPFC_OUTPUT_COUNT 8
#define ESPFC_OUTPUT_0 0
#define ESPFC_OUTPUT_1 2
#define ESPFC_OUTPUT_2 25
#define ESPFC_OUTPUT_3 26
#define ESPFC_OUTPUT_4 27
#define ESPFC_OUTPUT_5 12
#define ESPFC_OUTPUT_6 13
#define ESPFC_OUTPUT_7 14

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 35

#define ESPFC_SERIAL_COUNT 3
#define ESPFC_SERIAL_REMAP_PINS

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_TX 1
#define ESPFC_SERIAL_0_RX 3
#define ESPFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_1
#define ESPFC_SERIAL_1_TX 33
#define ESPFC_SERIAL_1_RX 32
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_RX_SERIAL)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_2
#define ESPFC_SERIAL_2_TX 17
#define ESPFC_SERIAL_2_RX 16
#define ESPFC_SERIAL_2_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_2_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_2_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_SOFT_0
#define ESPFC_SERIAL_SOFT_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_SOFT_0_WIFI

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN 4

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 36

#define ESPFC_ADC_1
#define ESPFC_ADC_1_PIN 39

#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_SOFTSERIAL | FEATURE_DYNAMIC_FILTER)

#define ESPFC_OUTPUT_PROTOCOL ESC_PROTOCOL_DISABLED

#define ESPFC_GUARD 0
#define ESPFC_GYRO_DENOM_MAX 1
//#define ESPFC_LOGGER_FS // doesn't compile on ESP32

#define ESPFC_FREE_RTOS
#ifndef CONFIG_FREERTOS_UNICORE
  #define ESPFC_MULTI_CORE
#endif

#define SERIAL_TX_FIFO_SIZE 0x7f

#define ESPFC_SERIAL_INIT(dev, sc, conf) \
  sc |= 0x8000000;\
  dev.begin(conf.baud, sc, conf.rx_pin, conf.tx_pin, conf.inverted);\

#define ESPFC_SPI_INIT(dev, sck, mosi, miso, ss) \
  dev.begin(sck, mosi, miso, ss); \

#if !defined(ESPFC_REVISION) // development build
  #undef ESPFC_OUTPUT_PROTOCOL
  #define ESPFC_OUTPUT_PROTOCOL ESC_PROTOCOL_DSHOT300

  #undef ESPFC_SERIAL_2_FN
  #define ESPFC_SERIAL_2_FN (SERIAL_FUNCTION_BLACKBOX)

  #undef ESPFC_SERIAL_2_BBAUD
  #define ESPFC_SERIAL_2_BBAUD (SERIAL_SPEED_250000)
#endif

namespace Espfc {

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

};
