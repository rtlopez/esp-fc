#include "Device/BusDevice.hpp"

namespace Espfc::Device {

const char** BusDevice::getNames()
{
  static const char* busDevChoices[] = {"NONE", "AUTO", "I2C", "SPI", "SLV", nullptr};
  return busDevChoices;
}

const char* BusDevice::getName(BusType type)
{
  if (type >= BUS_MAX) return "?";
  return getNames()[type];
}

} // namespace Espfc::Device
