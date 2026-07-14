#if defined(ESP32)

#include "Device/SerialDevice.h"
#include <Esp.h>

namespace {

static constexpr uint32_t SERIAL_UART_PARITY_NONE = 0B00000000;
static constexpr uint32_t SERIAL_UART_PARITY_EVEN = 0B00000010;
static constexpr uint32_t SERIAL_UART_PARITY_ODD = 0B00000011;

static constexpr uint32_t SERIAL_UART_NB_BIT_5 = 0B00000000;
static constexpr uint32_t SERIAL_UART_NB_BIT_6 = 0B00000100;
static constexpr uint32_t SERIAL_UART_NB_BIT_7 = 0B00001000;
static constexpr uint32_t SERIAL_UART_NB_BIT_8 = 0B00001100;

static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_0 = 0B00000000;
static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_1 = 0B00010000;
static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_15 = 0B00100000;
static constexpr uint32_t SERIAL_UART_NB_STOP_BIT_2 = 0B00110000;

} // namespace

namespace Espfc {

uint32_t targetSerialConfigFlags(const SerialDeviceConfig& conf)
{
  uint32_t sc = 0x8000000;
  // clang-format off
  switch(conf.data_bits)
  {
    case 8: sc |= SERIAL_UART_NB_BIT_8; break;
    case 7: sc |= SERIAL_UART_NB_BIT_7; break;
    case 6: sc |= SERIAL_UART_NB_BIT_6; break;
    case 5: sc |= SERIAL_UART_NB_BIT_5; break;
    default: sc |= SERIAL_UART_NB_BIT_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: sc |= SERIAL_UART_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  sc |= SERIAL_UART_PARITY_ODD;  break;
    default: sc |= SERIAL_UART_PARITY_NONE; break;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  sc |= SERIAL_UART_NB_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: sc |= SERIAL_UART_NB_STOP_BIT_15; break;
    case SDC_SERIAL_STOP_BITS_1:  sc |= SERIAL_UART_NB_STOP_BIT_1;  break;
    default: break;
  }
  // clang-format on
  return sc;
}

uint32_t getBoardId0()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)mac;
}

uint32_t getBoardId1()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)(mac >> 32);
}

uint32_t getBoardId2()
{
  return 0;
}

void targetReset()
{
  ESP.restart();
  while (1)
  {
  }
}

uint32_t targetCpuFreq()
{
  return ESP.getCpuFreqMHz();
}

uint32_t targetFreeHeap()
{
  return ESP.getFreeHeap();
}

} // namespace Espfc

#endif
