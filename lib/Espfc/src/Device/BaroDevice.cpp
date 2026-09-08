#include "BaroDevice.hpp"

namespace Espfc::Device {

const char** BaroDevice::getNames()
{
  static const char* devChoices[] = {"AUTO", "NONE", "BMP085", "MS5611", "BMP280", "SPL06-001", nullptr};
  return devChoices;
}

const char* BaroDevice::getName(DeviceType type)
{
  if (type >= BARO_MAX) return "?";
  return getNames()[type];
}

} // namespace Espfc::Device
