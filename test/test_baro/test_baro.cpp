#include "Device/Baro/BaroBMP280.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Hal;
using namespace Espfc::Device::Baro;

class MockBusDevice : public BusDevice
{
public:
  uint8_t readRegs[256] = {};  // registers ret
  uint8_t writeRegs[256] = {}; // registers captured by write
  int writeCalls = 0;
  bool failRead = false;

  BusType getType() const override
  {
    return BUS_I2C;
  }

  int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    if (failRead) return 0;
    for (uint8_t i = 0; i < length; i++)
      data[i] = readRegs[(regAddr + i) & 0xFF];
    return length;
  }

  int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    return read(devAddr, regAddr, length, data);
  }

  bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override
  {
    for (uint8_t i = 0; i < length; i++)
      writeRegs[(regAddr + i) & 0xFF] = data[i];
    writeCalls++;
    return true;
  }
};

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

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_baro_bmp280_caches_whoami);
  return UNITY_END();
}
