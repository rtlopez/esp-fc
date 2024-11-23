#include "Device/GyroDevice.h"
#include "Hal/Pgm.h"
#include <cstddef>

namespace Espfc::Device {

const char ** GyroDevice::getNames()
{
  static const char* devChoices[] = { PSTR("AUTO"), PSTR("NONE"), PSTR("MPU6000"), PSTR("MPU6050"), PSTR("MPU6500"), PSTR("MPU9250"), PSTR("LSM6DSO"), PSTR("ICM20602"),PSTR("BMI160"), NULL };
  return devChoices;
}

const char * GyroDevice::getName(DeviceType type)
{
  if(type >= GYRO_MAX) return PSTR("?");
  return getNames()[type];
}

}
