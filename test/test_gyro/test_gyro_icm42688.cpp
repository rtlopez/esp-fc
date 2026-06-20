#include <unity.h>
#include <ArduinoFake.h>
#include "Device/GyroICM42688.h"
#include "Device/GyroDevice.h"

using namespace Espfc;
using namespace Espfc::Device;

// Minimal BusDevice mock for unit testing the ICM-42688-P driver.
// Implements only the three pure-virtual methods; readByte/writeByte are
// inherited non-virtual helpers that delegate to read()/write().
class MockBusDevice : public BusDevice
{
public:
  uint8_t readRegs[256] = {};   // registers returned by read/readFast
  uint8_t writeRegs[256] = {};  // registers captured by write
  int writeCalls = 0;

  BusType getType() const override { return BUS_SPI; }

  int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    for (uint8_t i = 0; i < length; i++) data[i] = readRegs[(regAddr + i) & 0xFF];
    return length;
  }

  int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    return read(devAddr, regAddr, length, data);
  }

  bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override
  {
    for (uint8_t i = 0; i < length; i++) writeRegs[(regAddr + i) & 0xFF] = data[i];
    writeCalls++;
    return true;
  }
};

// U001: testConnection() returns true when WHO_AM_I == 0x47
void test_whoami_match()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  TEST_ASSERT_TRUE(dev.testConnection());
}

// U002: testConnection() returns false when WHO_AM_I != 0x47
void test_whoami_mismatch()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x12; // ICM-20602 WHO_AM_I
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  TEST_ASSERT_FALSE(dev.testConnection());
}

// U003: begin() returns 0 and issues no register writes when testConnection() fails
void test_begin_aborts_on_failed_connection()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x00;
  GyroICM42688 dev;
  int result = dev.begin(&bus, 0);
  TEST_ASSERT_EQUAL_INT(0, result);
  TEST_ASSERT_EQUAL_INT(0, bus.writeCalls);
}

// U004: readGyro() decodes big-endian 6-byte buffer correctly
void test_read_gyro_decoding()
{
  MockBusDevice bus;
  // gyro data registers start at 0x25; set big-endian pairs
  bus.readRegs[0x25] = 0x01; bus.readRegs[0x26] = 0x00; // X = 256
  bus.readRegs[0x27] = 0x02; bus.readRegs[0x28] = 0x00; // Y = 512
  bus.readRegs[0x29] = 0x03; bus.readRegs[0x2A] = 0x00; // Z = 768
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  VectorInt16 v;
  dev.readGyro(v);
  TEST_ASSERT_EQUAL_INT16(256, v.x);
  TEST_ASSERT_EQUAL_INT16(512, v.y);
  TEST_ASSERT_EQUAL_INT16(768, v.z);
}

// U005: readAccel() decodes big-endian 6-byte buffer correctly (incl. negative values)
void test_read_accel_decoding()
{
  MockBusDevice bus;
  // accel data registers start at 0x1F
  bus.readRegs[0x1F] = 0xFF; bus.readRegs[0x20] = 0xFE; // X = -2
  bus.readRegs[0x21] = 0x00; bus.readRegs[0x22] = 0x01; // Y = 1
  bus.readRegs[0x23] = 0x7F; bus.readRegs[0x24] = 0xFF; // Z = 32767
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  VectorInt16 v;
  dev.readAccel(v);
  TEST_ASSERT_EQUAL_INT16(-2,    v.x);
  TEST_ASSERT_EQUAL_INT16(1,     v.y);
  TEST_ASSERT_EQUAL_INT16(32767, v.z);
}

// U006: getType() returns GYRO_ICM42688
void test_get_type()
{
  GyroICM42688 dev;
  TEST_ASSERT_EQUAL_INT(GYRO_ICM42688, dev.getType());
}

// U007: getRate() returns 8000
void test_get_rate()
{
  GyroICM42688 dev;
  TEST_ASSERT_EQUAL_INT(8000, dev.getRate());
}

// U008: enum value and GYRO_MAX are correct at compile time
void test_enum_values()
{
  static_assert(GYRO_ICM42688 == 9, "GYRO_ICM42688 must equal 9 (append-only rule)");
  static_assert(GYRO_MAX == 10,     "GYRO_MAX must equal 10 after ICM42688 addition");
  TEST_PASS();
}

// U009: getName() returns "ICM42688" for GYRO_ICM42688
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
  RUN_TEST(test_begin_aborts_on_failed_connection);
  RUN_TEST(test_read_gyro_decoding);
  RUN_TEST(test_read_accel_decoding);
  RUN_TEST(test_get_type);
  RUN_TEST(test_get_rate);
  RUN_TEST(test_enum_values);
  RUN_TEST(test_name_table);
  return UNITY_END();
}
