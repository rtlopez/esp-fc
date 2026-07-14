#ifdef ARCH_RP2040

#include "TargetRP2040.h"
#include <RP2040Support.h>
#include <hardware/gpio.h>

namespace Espfc {

uint16_t targetSerialConfigFlags(const SerialDeviceConfig& conf)
{
  uint16_t flags = 0;
  // clang-format off
  switch(conf.data_bits)
  {
    case 8:  flags |= SERIAL_DATA_8; break;
    case 7:  flags |= SERIAL_DATA_7; break;
    case 6:  flags |= SERIAL_DATA_6; break;
    case 5:  flags |= SERIAL_DATA_5; break;
    default: flags |= SERIAL_DATA_8; break;
  }
  switch(conf.parity)
  {
    case SDC_SERIAL_PARITY_EVEN: flags |= SERIAL_PARITY_EVEN; break;
    case SDC_SERIAL_PARITY_ODD:  flags |= SERIAL_PARITY_ODD;  break;
    default: flags |= SERIAL_PARITY_NONE;
  }
  switch(conf.stop_bits)
  {
    case SDC_SERIAL_STOP_BITS_2:  flags |= SERIAL_STOP_BIT_2;  break;
    case SDC_SERIAL_STOP_BITS_15: flags |= SERIAL_STOP_BIT_1_5; break;
    case SDC_SERIAL_STOP_BITS_1:  flags |= SERIAL_STOP_BIT_1;  break;
    default: break;
  }
  // clang-format on
  return flags;
}

uint32_t getBoardId0()
{
  const char* id = rp2040.getChipID();
  return id[0] << 24 | id[1] << 16 | id[2] << 8 | id[3];
}

uint32_t getBoardId1()
{
  const char* id = rp2040.getChipID();
  return id[4] << 24 | id[5] << 16 | id[6] << 8 | id[7];
}

uint32_t getBoardId2()
{
  return 0;
}

void targetReset()
{
  watchdog_enable(1, 1);
  while (1)
  {
  }
}

uint32_t targetCpuFreq()
{
  return rp2040.f_cpu() / 1000000u;
}

uint32_t targetFreeHeap()
{
  return rp2040.getFreeHeap();
}

} // namespace Espfc

#endif
