#include "Device/Baro/BaroBMP280.hpp"
#include "MockBusDevice.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Hal;
using namespace Espfc::Device::Baro;

void test_baro_bmp280_caches_whoami()
{
  MockBusDevice bus;
  bus.readRegs[0xD0] = 0x58;
  BaroBMP280 dev;
  dev.setBus(&bus, 0x76);

  TEST_ASSERT_TRUE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x58, chipId.value());
}

void test_baro_device_get_name()
{
  using Espfc::Device::BaroDevice;
  TEST_ASSERT_EQUAL_STRING("AUTO", BaroDevice::getName(BARO_DEFAULT));
  TEST_ASSERT_EQUAL_STRING("NONE", BaroDevice::getName(BARO_NONE));
  TEST_ASSERT_EQUAL_STRING("?", BaroDevice::getName(BARO_MAX));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_baro_bmp280_caches_whoami);
  RUN_TEST(test_baro_device_get_name);
  return UNITY_END();
}
