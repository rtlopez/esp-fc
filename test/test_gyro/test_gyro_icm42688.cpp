#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Gyro/GyroICM42688.hpp"
#include "Device/GyroDevice.hpp"
#include "Device/Mag/MagHMC5883L.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Espfc::Device;
using namespace Espfc::Device::Gyro;
using namespace Espfc::Device::Mag;
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
    return BUS_SPI;
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

void test_whoami_match()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  TEST_ASSERT_TRUE(dev.testConnection());
}

void test_whoami_mismatch()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x12; // ICM-20602 WHO_AM_I
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  TEST_ASSERT_FALSE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x12, chipId.value());
}

void test_chip_id_is_empty_before_connection()
{
  GyroICM42688 dev;
  TEST_ASSERT_FALSE(dev.getChipId().has_value());
}

void test_chip_id_cached_on_success()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);

  TEST_ASSERT_TRUE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x47, chipId.value());
}

void test_chip_id_updated_on_mismatch()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);

  TEST_ASSERT_TRUE(dev.testConnection());
  bus.readRegs[0x75] = 0x12;
  TEST_ASSERT_FALSE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x12, chipId.value());
}

void test_chip_id_preserved_on_read_failure()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);

  TEST_ASSERT_TRUE(dev.testConnection());
  bus.failRead = true;
  TEST_ASSERT_FALSE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x47, chipId.value());
}

void test_mag_hmc5883l_uses_first_id_byte()
{
  MockBusDevice bus;
  bus.readRegs[0x0A] = 'H';
  bus.readRegs[0x0B] = '4';
  bus.readRegs[0x0C] = '3';
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);

  TEST_ASSERT_TRUE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8('H', chipId.value());
}

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

void test_begin_aborts_on_failed_connection()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x00;
  GyroICM42688 dev;
  int result = dev.begin(&bus, 0);
  TEST_ASSERT_EQUAL_INT(0, result);
  TEST_ASSERT_EQUAL_INT(0, bus.writeCalls);
}

void test_read_gyro_decoding()
{
  MockBusDevice bus;
  // gyro data registers start at 0x25
  bus.readRegs[0x25] = 0x01;
  bus.readRegs[0x26] = 0x00; // X = 256
  bus.readRegs[0x27] = 0x02;
  bus.readRegs[0x28] = 0x00; // Y = 512
  bus.readRegs[0x29] = 0x03;
  bus.readRegs[0x2A] = 0x00; // Z = 768
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  VectorInt16 v;
  dev.readGyro(v);
  TEST_ASSERT_EQUAL_INT16(256, v.x);
  TEST_ASSERT_EQUAL_INT16(512, v.y);
  TEST_ASSERT_EQUAL_INT16(768, v.z);
}

void test_read_accel_decoding()
{
  MockBusDevice bus;
  // accel data registers start at 0x1F
  bus.readRegs[0x1F] = 0xFF;
  bus.readRegs[0x20] = 0xFE; // X = -2
  bus.readRegs[0x21] = 0x00;
  bus.readRegs[0x22] = 0x01; // Y = 1
  bus.readRegs[0x23] = 0x7F;
  bus.readRegs[0x24] = 0xFF; // Z = 32767
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  VectorInt16 v;
  dev.readAccel(v);
  TEST_ASSERT_EQUAL_INT16(-2, v.x);
  TEST_ASSERT_EQUAL_INT16(1, v.y);
  TEST_ASSERT_EQUAL_INT16(32767, v.z);
}

void test_get_type()
{
  GyroICM42688 dev;
  TEST_ASSERT_EQUAL_INT(GYRO_ICM42688, dev.getType());
}

void test_get_rate()
{
  GyroICM42688 dev;
  TEST_ASSERT_EQUAL_INT(8000, dev.getRate());
}

void test_enum_values()
{
  static_assert(GYRO_ICM42688 == 9, "GYRO_ICM42688 must equal 9 (append-only rule)");
  static_assert(GYRO_MAX == 10, "GYRO_MAX must equal 10 after ICM42688 addition");
  TEST_PASS();
}

void test_name_table()
{
  const char* name = GyroDevice::getName(GYRO_ICM42688);
  TEST_ASSERT_EQUAL_STRING("ICM42688", name);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_whoami_match);
  RUN_TEST(test_whoami_mismatch);
  RUN_TEST(test_chip_id_is_empty_before_connection);
  RUN_TEST(test_chip_id_cached_on_success);
  RUN_TEST(test_chip_id_updated_on_mismatch);
  RUN_TEST(test_chip_id_preserved_on_read_failure);
  RUN_TEST(test_mag_hmc5883l_uses_first_id_byte);
  RUN_TEST(test_baro_bmp280_caches_whoami);
  RUN_TEST(test_begin_aborts_on_failed_connection);
  RUN_TEST(test_read_gyro_decoding);
  RUN_TEST(test_read_accel_decoding);
  RUN_TEST(test_get_type);
  RUN_TEST(test_get_rate);
  RUN_TEST(test_enum_values);
  RUN_TEST(test_name_table);
  return UNITY_END();
}
