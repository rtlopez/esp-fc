#if defined(ESP8266)

#include <Arduino.h>
#include <Esp.h>
#include <cstdint>

#include "Target/TargetESP8266.h"

namespace Espfc {

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
s.print("reset reason: ");
s.println(resetInfo->reason);

s.print("os s.print: ");
s.println(system_get_os_print());

//system_print_meminfo();

s.print("chip id: 0x");
s.println(system_get_chip_id(), HEX);

s.print("sdk version: ");
s.println(system_get_sdk_version());

s.print("boot version: ");
s.println(system_get_boot_version());

s.print("userbin addr: 0x");
s.println(system_get_userbin_addr(), HEX);

s.print("boot mode: ");
s.println(system_get_boot_mode() == 0 ? "SYS_BOOT_ENHANCE_MODE" : "SYS_BOOT_NORMAL_MODE");

s.print("flash size map: ");
s.println(system_get_flash_size_map());

s.print("time: ");
s.println(system_get_time() / 1000000);
*/

} // namespace Espfc

#endif
