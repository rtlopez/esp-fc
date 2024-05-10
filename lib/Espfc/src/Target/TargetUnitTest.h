#pragma once

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

#define ESPFC_GUARD 1

#define ESPFC_GYRO_I2C_RATE_MAX 2000
#define ESPFC_GYRO_SPI_RATE_MAX 8000

#define SERIAL_TX_FIFO_SIZE 0xFF

#define ESPFC_SERIAL_DEBUG_PORT 0

inline void targetReset()
{
}

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