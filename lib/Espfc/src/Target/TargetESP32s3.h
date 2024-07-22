#pragma once

#include "Esp.h"
#include "Debug_Espfc.h"

// pins to avoid:
// strapping: 0, 3, 45, 46
// flash/psram: 26-37 (reserved)
// usb/jtag: 19, 20

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 6 // ppm

#define ESPFC_OUTPUT_COUNT 4
#define ESPFC_OUTPUT_0 39
#define ESPFC_OUTPUT_1 40
#define ESPFC_OUTPUT_2 41
#define ESPFC_OUTPUT_3 42

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_DEV Serial0
#define ESPFC_SERIAL_0_DEV_T HardwareSerial
#define ESPFC_SERIAL_0_TX 43
#define ESPFC_SERIAL_0_RX 44
#define ESPFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_1
#define ESPFC_SERIAL_1_DEV Serial1
#define ESPFC_SERIAL_1_DEV_T HardwareSerial
#define ESPFC_SERIAL_1_TX 16
#define ESPFC_SERIAL_1_RX 15
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_2
#define ESPFC_SERIAL_2_DEV Serial2
#define ESPFC_SERIAL_2_DEV_T HardwareSerial
#define ESPFC_SERIAL_2_TX 18
#define ESPFC_SERIAL_2_RX 17
#define ESPFC_SERIAL_2_FN (SERIAL_FUNCTION_RX_SERIAL)
#define ESPFC_SERIAL_2_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_2_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_USB
#define ESPFC_SERIAL_USB_DEV Serial
#define ESPFC_SERIAL_USB_DEV_T HWCDC
#define ESPFC_SERIAL_USB_FN (SERIAL_FUNCTION_MSP)

#define ESPFC_SERIAL_SOFT_0
#define ESPFC_SERIAL_SOFT_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_SOFT_0_WIFI

#define ESPFC_SERIAL_REMAP_PINS
#define ESPFC_SERIAL_DEBUG_PORT SERIAL_USB
#define SERIAL_TX_FIFO_SIZE 0xFF

#define ESPFC_SPI_0
#define ESPFC_SPI_0_DEV SPI1
#define ESPFC_SPI_0_SCK 12
#define ESPFC_SPI_0_MOSI 11
#define ESPFC_SPI_0_MISO 13

#define ESPFC_SPI_CS_GYRO 8
#define ESPFC_SPI_CS_BARO 7

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 10
#define ESPFC_I2C_0_SDA 9
#define ESPFC_I2C_0_SOFT

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN 5

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 1

#define ESPFC_ADC_1
#define ESPFC_ADC_1_PIN 4

#define ESPFC_ADC_SCALE (3.3f / 4096)

#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_DYNAMIC_FILTER)

#define ESPFC_GUARD 0

#define ESPFC_GYRO_I2C_RATE_MAX 2000
#define ESPFC_GYRO_SPI_RATE_MAX 4000

#define ESPFC_DSHOT_TELEMETRY

#define ESPFC_FREE_RTOS
#ifndef CONFIG_FREERTOS_UNICORE
  #define ESPFC_MULTI_CORE
#endif

//#define ESPFC_FREE_RTOS_QUEUE
#define ESPFC_ATOMIC_QUEUE

#define ESPFC_DSP

#include "Device/SerialDevice.h"

#include "Target/TargetEsp32Common.h"

namespace Espfc {

template<>
inline int targetSerialInit(HWCDC& dev, const SerialDeviceConfig& conf)
{
  dev.begin(conf.baud);
  //dev.setTxTimeoutMs(10);
  //while(!dev) delay(10);
  return 1;
}

}
