// not used directly, but required so that the library dependency finder resolves include paths
// for lib/betaflight, same reason as in test_msp
#include <EscDriver.h>
#include <Gps.hpp>
#include <platform.h>

#include "Hal/ConfigStorage.hpp"
#include "Utils/Storage.hpp"
#include <unity.h>

using namespace Espfc;

static constexpr size_t STORAGE_SIZE = 2048;

// all instances share the same underlying memory, like a single physical eeprom
static Hal::ConfigStorage mem()
{
  Hal::ConfigStorage storage;
  storage.begin(STORAGE_SIZE);
  return storage;
}

void setUp()
{
  Hal::ConfigStorage storage = mem();
  for (size_t i = 0; i < STORAGE_SIZE; i++)
  {
    storage.writeByte(i, 0);
  }
}

void test_config_storage_size()
{
  Hal::ConfigStorage storage;

  TEST_ASSERT_EQUAL_UINT32(0, storage.size());
  TEST_ASSERT_TRUE(storage.begin(STORAGE_SIZE));
  TEST_ASSERT_EQUAL_UINT32(STORAGE_SIZE, storage.size());
}

void test_config_storage_byte_rw()
{
  Hal::ConfigStorage storage = mem();

  storage.writeByte(5, 0x42);
  TEST_ASSERT_EQUAL_UINT8(0x42, storage.readByte(5));
  TEST_ASSERT_EQUAL_UINT8(0, storage.readByte(6));
}

void test_config_storage_block_rw()
{
  Hal::ConfigStorage storage = mem();
  const uint8_t src[] = {1, 2, 3, 4};
  uint8_t dst[4] = {0};

  TEST_ASSERT_EQUAL_UINT32(4, storage.write(10, src, sizeof(src)));
  TEST_ASSERT_EQUAL_UINT32(4, storage.read(10, dst, sizeof(dst)));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(src, dst, sizeof(src));
}

void test_config_storage_out_of_bounds()
{
  Hal::ConfigStorage storage = mem();
  uint8_t buff[8] = {0};

  TEST_ASSERT_EQUAL_UINT32(0, storage.read(STORAGE_SIZE, buff, sizeof(buff)));
  TEST_ASSERT_EQUAL_UINT32(0, storage.write(STORAGE_SIZE, buff, sizeof(buff)));
  TEST_ASSERT_EQUAL_UINT32(4, storage.write(STORAGE_SIZE - 4, buff, sizeof(buff)));
  TEST_ASSERT_EQUAL_UINT8(0, storage.readByte(STORAGE_SIZE));
}

void test_storage_load_bad_magic()
{
  Utils::Storage storage;
  ModelConfig config;

  storage.begin();
  TEST_ASSERT_EQUAL_INT(STORAGE_ERR_BAD_MAGIC, storage.load(config));
}

void test_storage_save_load()
{
  Utils::Storage storage;
  ModelConfig saved;
  ModelConfig loaded;

  storage.begin();
  saved.pid[FC_PID_ROLL].P = 123;
  saved.input.rssiChannel = 7;

  TEST_ASSERT_EQUAL_INT(STORAGE_SAVE_SUCCESS, storage.save(saved));
  TEST_ASSERT_EQUAL_INT(STORAGE_LOAD_SUCCESS, storage.load(loaded));
  TEST_ASSERT_EQUAL_INT(123, loaded.pid[FC_PID_ROLL].P);
  TEST_ASSERT_EQUAL_INT(7, loaded.input.rssiChannel);
}

void test_storage_load_bad_version()
{
  Utils::Storage storage;
  ModelConfig config;

  storage.begin();
  storage.save(config);
  mem().writeByte(1, 0xff);

  TEST_ASSERT_EQUAL_INT(STORAGE_ERR_BAD_VERSION, storage.load(config));
}

void test_storage_load_bad_size()
{
  Utils::Storage storage;
  ModelConfig config;

  storage.begin();
  storage.save(config);
  mem().writeByte(2, 0x01);
  mem().writeByte(3, 0x00);

  TEST_ASSERT_EQUAL_INT(STORAGE_ERR_BAD_SIZE, storage.load(config));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_config_storage_size);
  RUN_TEST(test_config_storage_byte_rw);
  RUN_TEST(test_config_storage_block_rw);
  RUN_TEST(test_config_storage_out_of_bounds);

  RUN_TEST(test_storage_load_bad_magic);
  RUN_TEST(test_storage_save_load);
  RUN_TEST(test_storage_load_bad_version);
  RUN_TEST(test_storage_load_bad_size);

  return UNITY_END();
}
