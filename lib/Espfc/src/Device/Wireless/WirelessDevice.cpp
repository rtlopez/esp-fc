#include "WirelessDevice.hpp"

namespace Espfc {

namespace Device {

static const char* names[] = {
    "NONE",
    "AUTO",
    "NRF24L01",
};

const char** WirelessDevice::getNames()
{
  return names;
}

const char* WirelessDevice::getName(DeviceType type)
{
  if (type >= WIRELESS_MAX) return "UNKNOWN";
  return names[type];
}

} // namespace Device

} // namespace Espfc
