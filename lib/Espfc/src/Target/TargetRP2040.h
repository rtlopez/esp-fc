#pragma once

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN -1 // ppm

#define ESPFC_OUTPUT_COUNT 8
#define ESPFC_OUTPUT_0 2
#define ESPFC_OUTPUT_1 3
#define ESPFC_OUTPUT_2 4
#define ESPFC_OUTPUT_3 5
#define ESPFC_OUTPUT_4 -1
#define ESPFC_OUTPUT_5 -1
#define ESPFC_OUTPUT_6 -1
#define ESPFC_OUTPUT_7 -1

#define ESPFC_SERIAL_0
#define ESPFC_SERIAL_0_DEV Serial1
#define ESPFC_SERIAL_0_DEV_T SerialUART
#define ESPFC_SERIAL_0_TX 0
#define ESPFC_SERIAL_0_RX 1
#define ESPFC_SERIAL_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_0_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_0_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_1
#define ESPFC_SERIAL_1_DEV Serial2
#define ESPFC_SERIAL_1_DEV_T SerialUART
#define ESPFC_SERIAL_1_TX 8
#define ESPFC_SERIAL_1_RX 9
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_RX_SERIAL)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_USB
#define ESPFC_SERIAL_USB_DEV Serial
#define ESPFC_SERIAL_USB_DEV_T SerialUSB
#define ESPFC_SERIAL_USB_FN (SERIAL_FUNCTION_MSP)

#define ESPFC_SERIAL_REMAP_PINS
#define SERIAL_TX_FIFO_SIZE 128
#define ESPFC_SERIAL_DEBUG_PORT SERIAL_USB

#define ESPFC_SPI_0
#define ESPFC_SPI_0_DEV SPI1
#define ESPFC_SPI_0_DEV_T SPIClassRP2040
#define ESPFC_SPI_0_SCK 14
#define ESPFC_SPI_0_MOSI 15
#define ESPFC_SPI_0_MISO 12

#define ESPFC_SPI_CS_GYRO 13
#define ESPFC_SPI_CS_BARO 11

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SDA 16
#define ESPFC_I2C_0_SCL 17

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN -1

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 26

#define ESPFC_ADC_1
#define ESPFC_ADC_1_PIN 27

#define ESPFC_ADC_SCALE (3.3f / 4096)

#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_DYNAMIC_FILTER)

#define ESPFC_GUARD 0

#define ESPFC_GYRO_I2C_RATE_MAX 1000
#define ESPFC_GYRO_SPI_RATE_MAX 1000

#define ESPFC_MULTI_CORE
#define ESPFC_MULTI_CORE_RP2040

#include "Device/SerialDevice.h"
#include "Debug_Espfc.h"
#include <hardware/gpio.h>

namespace Espfc {

template<typename T>
inline int targetSerialInit(T& dev, const SerialDeviceConfig& conf)
{
  uint16_t sc = 0;
  switch(conf.data_bits)
  {
    case 8:  sc |= SERIAL_DATA_8; break;
    case 7:  sc |= SERIAL_DATA_7; break;
    case 6:  sc |= SERIAL_DATA_6; break;
    case 5:  sc |= SERIAL_DATA_5; break;
    default: sc |= SERIAL_DATA_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: sc |= SERIAL_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  sc |= SERIAL_PARITY_ODD;  break;
    default: sc |= SERIAL_PARITY_NONE;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  sc |= SERIAL_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: sc |= SERIAL_STOP_BIT_1_5; break;
    case SDC_SERIAL_STOP_BITS_1:  sc |= SERIAL_STOP_BIT_1;  break;
    default: break;
  }

  dev.setFIFOSize(SERIAL_TX_FIFO_SIZE);
  dev.setPinout(conf.tx_pin, conf.rx_pin);
  dev.begin(conf.baud, sc);

  if(conf.inverted) {
    gpio_set_inover(conf.rx_pin, GPIO_OVERRIDE_INVERT);
    gpio_set_outover(conf.tx_pin, GPIO_OVERRIDE_INVERT);
  }

  return 1;
}

template<>
inline int targetSerialInit(SerialUSB& dev, const SerialDeviceConfig& conf)
{
  dev.begin(conf.baud);
  //while(!dev) delay(10);
  return 1;
}

template<typename T>
inline int targetSPIInit(T& dev, int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  dev.setSCK(sck);
  dev.setRX(miso);
  dev.setTX(mosi);
  dev.begin();
  return 1;
}

template<typename T>
inline int targetI2CInit(T& dev, int8_t sda, int8_t scl, uint32_t speed)
{
  if(!dev.setSCL(scl)) return -1;
  if(!dev.setSDA(sda)) return -2;
  dev.setClock(speed);
  dev.begin();
  return 1;
}

inline uint32_t getBoardId0()
{
  const char * id = rp2040.getChipID();
  return id[0] << 24 | id[1] << 16 | id[2] << 8 | id[3];
}

inline uint32_t getBoardId1()
{
  const char * id = rp2040.getChipID();
  return id[4] << 24 | id[5] << 16 | id[6] << 8 | id[7];
}

inline uint32_t getBoardId2()
{
  return 0;
}

inline void targetReset()
{
  watchdog_enable(1, 1);
  while(1) {}
}

inline uint32_t targetCpuFreq()
{
  return rp2040.f_cpu() / 1000000u;
}

inline uint32_t targetFreeHeap()
{
  return rp2040.getFreeHeap();
}

};