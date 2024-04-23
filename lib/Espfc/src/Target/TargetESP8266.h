#pragma once

#include "Esp.h"

#define ESPFC_INPUT
#define ESPFC_INPUT_PIN 13  // D7 - ppm

#define ESPFC_OUTPUT_COUNT 4 // 4 is minimum
#define ESPFC_OUTPUT_0 0   // D3
#define ESPFC_OUTPUT_1 14  // D5
#define ESPFC_OUTPUT_2 12  // D6
#define ESPFC_OUTPUT_3 15  // D8

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
#define ESPFC_SERIAL_1_TX 2  // D4
#define ESPFC_SERIAL_1_RX -1
#define ESPFC_SERIAL_1_FN (SERIAL_FUNCTION_NONE)
#define ESPFC_SERIAL_1_BAUD (SERIAL_SPEED_115200)
#define ESPFC_SERIAL_1_BBAUD (SERIAL_SPEED_NONE)

#define ESPFC_SERIAL_SOFT_0
#define ESPFC_SERIAL_SOFT_0_FN (SERIAL_FUNCTION_MSP)
#define ESPFC_SERIAL_SOFT_0_WIFI

#define ESPFC_SERIAL_DEBUG_PORT SERIAL_UART_0
#define SERIAL_TX_FIFO_SIZE UART_TX_FIFO_SIZE

#define ESPFC_I2C_0
#define ESPFC_I2C_0_SCL 5  // D1
#define ESPFC_I2C_0_SDA 4  // D2
#define ESPFC_I2C_0_SOFT

#define ESPFC_BUZZER
#define ESPFC_BUZZER_PIN 16  // D0

#define ESPFC_ADC_0
#define ESPFC_ADC_0_PIN 17   // A0

#define ESPFC_ADC_SCALE (1.0f / 1024)

#define ESPFC_FEATURE_MASK (FEATURE_RX_PPM | FEATURE_DYNAMIC_FILTER)

#define ESPFC_GUARD 1

#define ESPFC_GYRO_I2C_RATE_MAX 1000
#define ESPFC_GYRO_SPI_RATE_MAX 1000

#define ESPFC_WIFI_ALT
#define ESPFC_ESPNOW

#include "Device/SerialDevice.h"

namespace Espfc {

template<typename T>
inline int targetSerialInit(T& dev, const SerialDeviceConfig& conf)
{
  uint32_t sc = 0;
  switch(conf.data_bits)
  {
    case 8: sc |= UART_NB_BIT_8; break;
    case 7: sc |= UART_NB_BIT_7; break;
    case 6: sc |= UART_NB_BIT_6; break;
    case 5: sc |= UART_NB_BIT_5; break;
    default: sc |= UART_NB_BIT_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: sc |= UART_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  sc |= UART_PARITY_ODD;  break;
    default: sc |= UART_PARITY_NONE;  break;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  sc |= UART_NB_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: sc |= UART_NB_STOP_BIT_15; break;
    case SDC_SERIAL_STOP_BITS_1:  sc |= UART_NB_STOP_BIT_1;  break;
    default: break;
  }

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

inline uint32_t getBoardId0()
{
  return ESP.getChipId();
}

inline uint32_t getBoardId1()
{
  return ESP.getFlashChipId();
}

inline uint32_t getBoardId2()
{
  return ESP.getFlashChipSize();
}

inline void targetReset()
{
  // pin setup to ensure boot from flash
  pinMode(0, OUTPUT); digitalWrite(0, HIGH); // GPIO0 to HI
  pinMode(2, OUTPUT); digitalWrite(2, HIGH); // GPIO2 to HI
  pinMode(15, OUTPUT); digitalWrite(15, LOW); // GPIO15 to LO
  pinMode(0, INPUT);
  pinMode(2, INPUT);
  pinMode(15, INPUT);
  ESP.reset();
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

/*
//#include "user_interface.h"
const rst_info * resetInfo = system_get_rst_info();
s.print(F("reset reason: "));
s.println(resetInfo->reason);

s.print(F("os s.print: "));
s.println(system_get_os_print());

//system_print_meminfo();

s.print(F("chip id: 0x"));
s.println(system_get_chip_id(), HEX);

s.print(F("sdk version: "));
s.println(system_get_sdk_version());

s.print(F("boot version: "));
s.println(system_get_boot_version());

s.print(F("userbin addr: 0x"));
s.println(system_get_userbin_addr(), HEX);

s.print(F("boot mode: "));
s.println(system_get_boot_mode() == 0 ? F("SYS_BOOT_ENHANCE_MODE") : F("SYS_BOOT_NORMAL_MODE"));

s.print(F("flash size map: "));
s.println(system_get_flash_size_map());

s.print(F("time: "));
s.println(system_get_time() / 1000000);
*/

};