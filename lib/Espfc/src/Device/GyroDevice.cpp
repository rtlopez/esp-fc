#include "Device/GyroDevice.hpp"

namespace Espfc::Device {

const char** GyroDevice::getNames()
{
  static const char* devChoices[] = {"NONE",    "AUTO",     "MPU6000", "MPU6050",  "MPU6500", "MPU9250",
                                     "LSM6DSO", "ICM20602", "BMI160",  "ICM42688", nullptr};
  return devChoices;
}

const char* GyroDevice::getName(DeviceType type)
{
  if (type >= GYRO_MAX) return "?";
  return getNames()[type];
}

} // namespace Espfc::Device
