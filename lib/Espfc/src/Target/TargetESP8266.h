#pragma once

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 13 // D7 - ppm

#define ESPFC_OUTPUT_COUNT 4 // 4 is minimum
#define ESPFC_OUTPUT_0 0     // D3
#define ESPFC_OUTPUT_1 14    // D5
#define ESPFC_OUTPUT_2 12    // D6
#define ESPFC_OUTPUT_3 15    // D8

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_DEV Serial
#define ESPFC_SERIAL_0_DEV_T HardwareSerial
#define ESPFC_SERIAL_0_TX 1
#define ESPFC_SERIAL_0_RX 3
#define ESPFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_1
#define ESPFC_SERIAL_1_DEV Serial1
#define ESPFC_SERIAL_1_DEV_T HardwareSerial
#define ESPFC_SERIAL_1_TX 2 // D4
#define ESPFC_SERIAL_1_RX -1
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_NONE)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_SOFT_0
#define ESPFC_SERIAL_SOFT_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_SOFT_0_WIFI

#define ESPFC_SERIAL_DEBUG_PORT SERIAL_UART_0

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 5 // D1
#define ESPFC_I2C_0_SDA 4 // D2
#define ESPFC_I2C_0_SOFT

#define ESPFC_BUZZER_PIN 16 // D0
#define ESPFC_BUTTON_PIN -1
#define ESPFC_LED_PIN -1

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 17 // A0

#define ESPFC_ADC_SCALE (1.0f / 1024)

#define ESPFC_FEATURE_MASK (FEATURE_RX_PPM | FEATURE_DYNAMIC_FILTER)

#define ESPFC_GYRO_I2C_RATE_MAX 1000
#define ESPFC_GYRO_SPI_RATE_MAX 1000

#define ESPFC_WIFI_ALT
#define ESPFC_ESPNOW

#include "Device/SerialDevice.h"
#include <uart.h>

namespace Espfc {

uint32_t targetSerialConfigFlags(const SerialDeviceConfig& conf);

constexpr size_t targetSerialTxBufferSize()
{
  return UART_TX_FIFO_SIZE;
}

template<typename T>
inline int targetSerialInit(T& dev, const SerialDeviceConfig& conf)
{
  uint32_t sc = targetSerialConfigFlags(conf);
  const bool isUart0 = &dev == &Serial;
  dev.begin(conf.baud, (SerialConfig)sc, isUart0 ? SERIAL_FULL : SERIAL_TX_ONLY, isUart0 ? 1 : 2, conf.inverted);

  return 1;
}

template<typename T>
inline int targetSPIInit(T& dev, int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  dev.pins(sck, miso, mosi, ss);
  dev.begin();
  return 1;
}

template<typename T>
inline int targetI2CInit(T& dev, int8_t sda, int8_t scl, uint32_t speed)
{
  dev.begin(sda, scl, speed);
  return 1;
}

uint32_t getBoardId0();
uint32_t getBoardId1();
uint32_t getBoardId2();

void targetReset();

uint32_t targetCpuFreq();
uint32_t targetFreeHeap();

} // namespace Espfc
