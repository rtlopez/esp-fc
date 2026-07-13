#pragma once

#include "Device/SerialDevice.h"

#define ESPFC_WIFI
#define ESPFC_ESPNOW
#define ESPFC_LED_WS2812

namespace Espfc {

uint32_t targetSerialConfigFlags(const SerialDeviceConfig& conf);

constexpr size_t targetSerialTxBufferSize()
{
  return 0xFF;
}

template<typename T>
inline int targetSerialInit(T& dev, const SerialDeviceConfig& conf)
{
  uint32_t sc = targetSerialConfigFlags(conf);
  if (dev) dev.end();
  dev.setTxBufferSize(targetSerialTxBufferSize());
  dev.begin(conf.baud, sc, conf.rx_pin, conf.tx_pin, conf.inverted);
  return 1;
}

template<typename T>
inline int targetSPIInit(T& dev, int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  dev.begin(sck, miso, mosi, ss);
  return 1;
}

template<typename T>
inline int targetI2CInit(T& dev, int8_t sda, int8_t scl, uint32_t speed)
{
  dev.begin(sda, scl, speed);
  dev.setTimeout(50);
  return 1;
}

void targetReset();

uint32_t getBoardId0();
uint32_t getBoardId1();
uint32_t getBoardId2();

uint32_t targetCpuFreq();
uint32_t targetFreeHeap();

}; // namespace Espfc
