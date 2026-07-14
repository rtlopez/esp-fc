#if defined(ESP8266)

#include <Arduino.h>
#include <Esp.h>
#include <cstdint>

#include "Target/TargetESP8266.h"

namespace Espfc {

uint32_t targetSerialConfigFlags(const SerialDeviceConfig& conf)
{
  uint32_t sc = 0;
  // clang-format off
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
  // clang-format on
  return sc;
}

uint32_t getBoardId0()
{
  return ESP.getChipId();
}

uint32_t getBoardId1()
{
  return ESP.getFlashChipId();
}

uint32_t getBoardId2()
{
  return ESP.getFlashChipSize();
}

void targetReset()
{
  // pin setup to ensure boot from flash
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH); // GPIO0 to HI
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // GPIO2 to HI
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW); // GPIO15 to LO
  pinMode(0, INPUT);
  pinMode(2, INPUT);
  pinMode(15, INPUT);
  ESP.reset();
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

} // namespace Espfc

#endif
