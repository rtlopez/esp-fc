#include "Device/MagDevice.hpp"

namespace Espfc::Device {

const char** MagDevice::getNames()
{
  static const char* devChoices[] = {"AUTO", "NONE", "HMC5883L", "AK8975", "AK8963", "QMC5883L", "QMC5883P", nullptr};
  return devChoices;
}

const char* MagDevice::getName(DeviceType type)
{
  if (type >= MAG_MAX) return "?";
  return getNames()[type];
}

} // namespace Espfc::Device
