#pragma once

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

inline void targetReset()
{
}
