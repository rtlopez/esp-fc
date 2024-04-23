#pragma once

#include "Esp.h"
#include "Debug_Espfc.h"
// flash pins: 12-17 (reserved)
// usb pins: 18,19
// strapping pins: 2,8,9, must be high on boot
#define ESPFC_INPUT
#define ESPFC_INPUT_PIN -1 // ppm

#define ESPFC_OUTPUT_COUNT ESC_CHANNEL_COUNT
#define ESPFC_OUTPUT_0 2
#define ESPFC_OUTPUT_1 3
#define ESPFC_OUTPUT_2 4
#define ESPFC_OUTPUT_3 5

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_DEV Serial0
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

#define ESPFC_SERIAL_USB
#define ESPFC_SERIAL_USB_DEV Serial
#define ESPFC_SERIAL_USB_DEV_T HWCDC
#define ESPFC_SERIAL_USB_FN (SERIAL_FUNCTION_MSP)

#define ESPFC_SERIAL_SOFT_0
#define ESPFC_SERIAL_SOFT_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_SOFT_0_WIFI

#define ESPFC_SERIAL_REMAP_PINS
#define ESPFC_SERIAL_DEBUG_PORT SERIAL_UART_0
#define SERIAL_TX_FIFO_SIZE 0xFF

#define ESPFC_SPI_0
#define ESPFC_SPI_0_DEV SPI1
#define ESPFC_SPI_0_SCK -1
#define ESPFC_SPI_0_MOSI -1
#define ESPFC_SPI_0_MISO -1

#define ESPFC_SPI_CS_GYRO -1
#define ESPFC_SPI_CS_BARO -1

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 6
#define ESPFC_I2C_0_SDA 8
#define ESPFC_I2C_0_SOFT

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN -1

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 0

#define ESPFC_ADC_1
#define ESPFC_ADC_1_PIN 1

#define ESPFC_ADC_SCALE (3.3f / 4096)

#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_DYNAMIC_FILTER)

#define ESPFC_GUARD 0

#define ESPFC_GYRO_I2C_RATE_MAX 1000
#define ESPFC_GYRO_SPI_RATE_MAX 2000

#define ESPFC_DSP

#include "Device/SerialDevice.h"

#include "Target/TargetEsp32Common.h"

namespace Espfc {

template<>
inline int targetSerialInit(HWCDC& dev, const SerialDeviceConfig& conf)
{
  dev.begin(conf.baud);
  //while(!dev) delay(10);
  return 1;
}

}
