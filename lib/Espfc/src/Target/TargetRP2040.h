#pragma once

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 7 // ppm

#define ESPFC_OUTPUT_COUNT 8
#define ESPFC_OUTPUT_0 10
#define ESPFC_OUTPUT_1 11
#define ESPFC_OUTPUT_2 12
#define ESPFC_OUTPUT_3 13
#define ESPFC_OUTPUT_4 -1
#define ESPFC_OUTPUT_5 -1
#define ESPFC_OUTPUT_6 -1
#define ESPFC_OUTPUT_7 -1

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_TX 0
#define ESPFC_SERIAL_0_RX 1
#define ESPFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_1
#define ESPFC_SERIAL_1_TX 20
#define ESPFC_SERIAL_1_RX 21
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_RX_SERIAL)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_REMAP_PINS
#define SERIAL_TX_FIFO_SIZE 32

#define ESPFC_SPI_0
#define ESPFC_SPI_0_SCK 2
#define ESPFC_SPI_0_MOSI 3
#define ESPFC_SPI_0_MISO 4

#define ESPFC_SPI_CS_GYRO 5
#define ESPFC_SPI_CS_BARO 6

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 8
#define ESPFC_I2C_0_SDA 9

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN 4

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 26

#define ESPFC_ADC_1
#define ESPFC_ADC_1_PIN 27

#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_SOFTSERIAL | FEATURE_DYNAMIC_FILTER)

#define ESPFC_OUTPUT_PROTOCOL ESC_PROTOCOL_DISABLED

#define ESPFC_GUARD 0
#define ESPFC_GYRO_DENOM_MAX 1

#define ESPFC_SERIAL_INIT(dev, sc, conf) \
  /*dev.setPinout(conf.tx_pin, conf.rx_pin);*/ \
  dev.begin(conf.baud); \
  //TODO: inverted

#define ESPFC_SPI_INIT(dev, sck, mosi, miso, ss) \
  dev.setSCK(sck); \
  dev.setRX(miso); \
  dev.setTX(mosi); \
  dev.begin(); \

namespace Espfc {

inline uint32_t getBoardId0()
{
  return 0;
}

inline uint32_t getBoardId1()
{
  return 0;
}

inline uint32_t getBoardId2()
{
  return 0;
}

inline void targetReset()
{
  while(1) {}
}

inline uint32_t targetCpuFreq()
{
  return 0;
}

inline uint32_t targetFreeHeap()
{
  return 0;
}

};