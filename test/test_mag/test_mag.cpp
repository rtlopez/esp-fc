#include "Device/Mag/MagHMC5883L.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Hal;
using namespace Espfc::Device::Mag;

class MockBusDevice : public BusDevice
{
public:
  uint8_t readRegs[256] = {};
  uint8_t writeRegs[256] = {};
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

static void setHmcId(MockBusDevice& bus)
{
  bus.readRegs[0x0A] = 'H';
  bus.readRegs[0x0B] = '4';
  bus.readRegs[0x0C] = '3';
}

void test_mag_hmc5883l_whoami()
{
  MockBusDevice bus;
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);

  TEST_ASSERT_FALSE(dev.testConnection());

  setHmcId(bus);
  TEST_ASSERT_TRUE(dev.testConnection());

  bus.readRegs[0x0C] = '4';
  TEST_ASSERT_FALSE(dev.testConnection());
}

void test_mag_hmc5883l_chip_id()
{
  MockBusDevice bus;
  setHmcId(bus);
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);

  TEST_ASSERT_FALSE(dev.getChipId().has_value());
  TEST_ASSERT_TRUE(dev.testConnection());
  TEST_ASSERT_EQUAL_HEX8('H', dev.getChipId().value());
}

void test_mag_hmc5883l_connection_read_failure()
{
  MockBusDevice bus;
  setHmcId(bus);
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);

  bus.failRead = true;
  TEST_ASSERT_FALSE(dev.testConnection());
}

void test_mag_hmc5883l_begin_configures()
{
  MockBusDevice bus;
  setHmcId(bus);
  bus.writeRegs[0x00] = 0xFF;
  bus.writeRegs[0x01] = 0xFF;
  bus.writeRegs[0x02] = 0xFF;
  MagHMC5883L dev;

  TEST_ASSERT_EQUAL_INT(1, dev.begin(&bus));
  TEST_ASSERT_EQUAL_HEX8(0x18, bus.writeRegs[0x00]); // 1 sample, 75 Hz, normal bias
  TEST_ASSERT_EQUAL_HEX8(0x20, bus.writeRegs[0x01]); // gain 1090
  TEST_ASSERT_EQUAL_HEX8(0x00, bus.writeRegs[0x02]); // continuous mode
  TEST_ASSERT_EQUAL_INT(3, bus.writeCalls);
}

void test_mag_hmc5883l_begin_fails_without_device()
{
  MockBusDevice bus;
  MagHMC5883L dev;

  TEST_ASSERT_EQUAL_INT(0, dev.begin(&bus));
  TEST_ASSERT_EQUAL_INT(0, bus.writeCalls);
}

void test_mag_hmc5883l_read_decoding()
{
  MockBusDevice bus;
  bus.readRegs[0x03] = 0x01; // X
  bus.readRegs[0x04] = 0x02;
  bus.readRegs[0x05] = 0xFF; // Z
  bus.readRegs[0x06] = 0xFE;
  bus.readRegs[0x07] = 0x80; // Y
  bus.readRegs[0x08] = 0x00;
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);

  VectorInt16 v;
  TEST_ASSERT_EQUAL_INT(1, dev.readMag(v));
  TEST_ASSERT_EQUAL_INT16(0x0102, v.x);
  TEST_ASSERT_EQUAL_INT16(-32768, v.y);
  TEST_ASSERT_EQUAL_INT16(-2, v.z);
}

void test_mag_hmc5883l_read_failure_keeps_value()
{
  MockBusDevice bus;
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);
  bus.failRead = true;

  VectorInt16 v(1, 2, 3);
  TEST_ASSERT_EQUAL_INT(0, dev.readMag(v));
  TEST_ASSERT_EQUAL_INT16(1, v.x);
  TEST_ASSERT_EQUAL_INT16(2, v.y);
  TEST_ASSERT_EQUAL_INT16(3, v.z);
}

void test_mag_hmc5883l_convert()
{
  MagHMC5883L dev;
  const VectorFloat f = dev.convert(VectorInt16(1090, -545, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, f.x);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.5f, f.y);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, f.z);
}

void test_mag_hmc5883l_type_and_rate()
{
  MagHMC5883L dev;
  TEST_ASSERT_EQUAL_INT(MAG_HMC5883L, dev.getType());
  TEST_ASSERT_EQUAL_INT(75, dev.getRate());
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_mag_hmc5883l_whoami);
  RUN_TEST(test_mag_hmc5883l_chip_id);
  RUN_TEST(test_mag_hmc5883l_connection_read_failure);
  RUN_TEST(test_mag_hmc5883l_begin_configures);
  RUN_TEST(test_mag_hmc5883l_begin_fails_without_device);
  RUN_TEST(test_mag_hmc5883l_read_decoding);
  RUN_TEST(test_mag_hmc5883l_read_failure_keeps_value);
  RUN_TEST(test_mag_hmc5883l_convert);
  RUN_TEST(test_mag_hmc5883l_type_and_rate);
  return UNITY_END();
}
