#pragma once

#include <cstddef>
#include <cstdint>

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 0

#define ESPFC_OUTPUT_COUNT 4 // 4 is minimum
#define ESPFC_OUTPUT_0 1
#define ESPFC_OUTPUT_1 2
#define ESPFC_OUTPUT_2 3
#define ESPFC_OUTPUT_3 4

#define ESPFC_OUTPUT_PROTOCOL ESC_PROTOCOL_DISABLED
#define ESPFC_FEATURE_MASK (0)

#define ESPFC_GYRO_I2C_RATE_MAX 2000
#define ESPFC_GYRO_SPI_RATE_MAX 8000

#define ESPFC_SPI_0
#define ESPFC_SPI_0_SCK -1
#define ESPFC_SPI_0_MOSI -1
#define ESPFC_SPI_0_MISO -1

#define ESPFC_SPI_CS_GYRO -1
#define ESPFC_SPI_CS_BARO -1

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SDA -1
#define ESPFC_I2C_0_SCL -1

#define ESPFC_SERIAL_DEBUG_PORT 0
#define ESPFC_BUZZER_PIN -1
#define ESPFC_BUTTON_PIN -1
#define ESPFC_LED_PIN -1

constexpr size_t targetSerialTxBufferSize()
{
  return 0xff;
}

inline void targetReset() {}

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

inline uint32_t targetCpuFreq()
{
  return 1;
}

inline uint32_t targetFreeHeap()
{
  return 1;
}