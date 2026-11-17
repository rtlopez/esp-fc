#include "Connect/VtxSmartAudio.hpp"
#include "Connect/VtxTramp.hpp"
#include "Stream/ReadWritable.hpp"
#include "Utils/Crc.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>
#include <vector>

using namespace Espfc;
using namespace Espfc::Connect;

class TestSerial : public Stream::ReadWritable
{
public:
  void begin(const Hal::SerialDeviceConfig&) override {}
  void updateBaudRate(int) override {}
  int available() override { return 0; }
  int read() override { return -1; }
  size_t readMany(uint8_t*, size_t) override { return 0; }
  int peek() override { return -1; }
  size_t write(uint8_t c) override
  {
    data.push_back(c);
    return 1;
  }
  size_t write(const uint8_t* d, size_t len) override
  {
    data.insert(data.end(), d, d + len);
    return len;
  }
  int availableForWrite() override { return space; }
  void flush() override {}
  bool isTxFifoEmpty() override { return true; }

  std::vector<uint8_t> data;
  int space = 1024;
};

void test_vtx_tramp_encode()
{
  uint8_t f[VtxTramp::FRAME_SIZE];
  VtxTramp::encode(f, 'F', 5800);
  TEST_ASSERT_EQUAL_UINT8(0x0F, f[0]);
  TEST_ASSERT_EQUAL_UINT8('F', f[1]);
  TEST_ASSERT_EQUAL_UINT8(5800 & 0xFF, f[2]);
  TEST_ASSERT_EQUAL_UINT8(5800 >> 8, f[3]);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>('F' + (5800 & 0xFF) + (5800 >> 8)), f[14]);
  TEST_ASSERT_EQUAL_UINT8(0, f[15]);
}

void test_vtx_tramp_tables()
{
  TEST_ASSERT_EQUAL_UINT16(5865, VtxTramp::frequency(1, 1));
  TEST_ASSERT_EQUAL_UINT16(5917, VtxTramp::frequency(5, 8));
  TEST_ASSERT_EQUAL_UINT16(5865, VtxTramp::frequency(0, 0));
  TEST_ASSERT_EQUAL_UINT16(5917, VtxTramp::frequency(200, 200));
  TEST_ASSERT_EQUAL_UINT16(25, VtxTramp::power(0));
  TEST_ASSERT_EQUAL_UINT16(25, VtxTramp::power(1));
  TEST_ASSERT_EQUAL_UINT16(600, VtxTramp::power(5));
  TEST_ASSERT_EQUAL_UINT16(600, VtxTramp::power(99));
}

void test_vtx_tramp_update_cycle()
{
  Model model;
  TestSerial serial;
  VtxTramp vtx(model);
  model.config.vtx.band = 4;
  model.config.vtx.channel = 2;
  model.config.vtx.power = 3;
  model.config.vtx.pitMode = 1;
  vtx.begin(&serial);
  TEST_ASSERT_TRUE(model.state.vtx.active);
  TEST_ASSERT_EQUAL_UINT8(VTXDEV_TRAMP, model.state.vtx.type);

  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(16, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8('F', serial.data[1]);
  TEST_ASSERT_EQUAL_UINT8(5760 & 0xFF, serial.data[2]);

  vtx.update(1001);
  TEST_ASSERT_EQUAL_size_t(16, serial.data.size());

  vtx.update(1000 + 200000);
  TEST_ASSERT_EQUAL_size_t(32, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8('P', serial.data[17]);
  TEST_ASSERT_EQUAL_UINT8(200, serial.data[18]);

  vtx.update(1000 + 400000);
  TEST_ASSERT_EQUAL_size_t(48, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8('I', serial.data[33]);
  TEST_ASSERT_EQUAL_UINT8(0, serial.data[34]);
}

void test_vtx_tramp_low_power_disarm()
{
  Model model;
  TestSerial serial;
  VtxTramp vtx(model);
  model.config.vtx.power = 5;
  model.config.vtx.lowPowerDisarm = 1;
  vtx.begin(&serial);
  vtx.update(1000);
  vtx.update(1000 + 200000);
  TEST_ASSERT_EQUAL_UINT8('P', serial.data[17]);
  TEST_ASSERT_EQUAL_UINT8(25, serial.data[18]);
}

void test_vtx_tramp_tx_full_skips_frame()
{
  Model model;
  TestSerial serial;
  VtxTramp vtx(model);
  vtx.begin(&serial);
  serial.space = 4;
  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(0, serial.data.size());
}

void test_vtx_smartaudio_encode()
{
  uint8_t f[VtxSmartAudio::FRAME_SIZE];
  VtxSmartAudio::encodeChannel(f, 2, 3);
  TEST_ASSERT_EQUAL_UINT8(0xAA, f[0]);
  TEST_ASSERT_EQUAL_UINT8(0x55, f[1]);
  TEST_ASSERT_EQUAL_UINT8(0x07, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0x01, f[3]);
  TEST_ASSERT_EQUAL_UINT8(10, f[4]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 5), f[5]);

  VtxSmartAudio::encodeChannel(f, 0, 0);
  TEST_ASSERT_EQUAL_UINT8(0, f[4]);
  VtxSmartAudio::encodeChannel(f, 99, 99);
  TEST_ASSERT_EQUAL_UINT8(39, f[4]);

  VtxSmartAudio::encodePower(f, 0);
  TEST_ASSERT_EQUAL_UINT8(0x05, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0, f[4]);
  VtxSmartAudio::encodePower(f, 3);
  TEST_ASSERT_EQUAL_UINT8(2, f[4]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 5), f[5]);
}

void test_vtx_direct_frequency()
{
  VtxConfig c;
  TEST_ASSERT_EQUAL_UINT16(0, vtxDirectFrequency(c));
  c.band = 0;
  TEST_ASSERT_EQUAL_UINT16(0, vtxDirectFrequency(c));
  c.frequency = 5800;
  TEST_ASSERT_EQUAL_UINT16(5800, vtxDirectFrequency(c));
  c.frequency = 9999;
  TEST_ASSERT_EQUAL_UINT16(5999, vtxDirectFrequency(c));
  c.frequency = 100;
  TEST_ASSERT_EQUAL_UINT16(5000, vtxDirectFrequency(c));
  c.band = 1;
  c.frequency = 5800;
  TEST_ASSERT_EQUAL_UINT16(0, vtxDirectFrequency(c));
  c.pitModeFrequency = 5584;
  TEST_ASSERT_EQUAL_UINT16(0, vtxDirectFrequency(c));
  c.pitMode = 1;
  TEST_ASSERT_EQUAL_UINT16(5584, vtxDirectFrequency(c));
}

void test_vtx_tramp_frequency_mode()
{
  Model model;
  TestSerial serial;
  VtxTramp vtx(model);
  model.config.vtx.band = 0;
  model.config.vtx.frequency = 5801;
  vtx.begin(&serial);
  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(16, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8('F', serial.data[1]);
  TEST_ASSERT_EQUAL_UINT8(5801 & 0xFF, serial.data[2]);
  TEST_ASSERT_EQUAL_UINT8(5801 >> 8, serial.data[3]);
}

void test_vtx_tramp_frequency_unset_skips()
{
  Model model;
  TestSerial serial;
  VtxTramp vtx(model);
  model.config.vtx.band = 0;
  vtx.begin(&serial);
  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(0, serial.data.size());
}

void test_vtx_tramp_pit_mode_frequency()
{
  Model model;
  TestSerial serial;
  VtxTramp vtx(model);
  model.config.vtx.pitMode = 1;
  model.config.vtx.pitModeFrequency = 5584;
  vtx.begin(&serial);
  vtx.update(1000);
  TEST_ASSERT_EQUAL_UINT8(5584 & 0xFF, serial.data[2]);
  TEST_ASSERT_EQUAL_UINT8(5584 >> 8, serial.data[3]);
}

void test_vtx_smartaudio_encode_frequency()
{
  uint8_t f[VtxSmartAudio::FRAME_SIZE];
  const size_t len = VtxSmartAudio::encodeFrequency(f, 5800);
  TEST_ASSERT_EQUAL_size_t(7, len);
  TEST_ASSERT_EQUAL_UINT8(0xAA, f[0]);
  TEST_ASSERT_EQUAL_UINT8(0x55, f[1]);
  TEST_ASSERT_EQUAL_UINT8(0x09, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0x02, f[3]);
  TEST_ASSERT_EQUAL_UINT8(5800 >> 8, f[4]);
  TEST_ASSERT_EQUAL_UINT8(5800 & 0xFF, f[5]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 6), f[6]);
}

void test_vtx_smartaudio_frequency_mode()
{
  Model model;
  TestSerial serial;
  VtxSmartAudio vtx(model);
  model.config.vtx.band = 0;
  model.config.vtx.frequency = 5800;
  vtx.begin(&serial);
  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(8, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x09, serial.data[3]);
}

void test_vtx_smartaudio_update_cycle()
{
  Model model;
  TestSerial serial;
  VtxSmartAudio vtx(model);
  model.config.vtx.band = 1;
  model.config.vtx.channel = 8;
  model.config.vtx.power = 2;
  vtx.begin(&serial);
  TEST_ASSERT_EQUAL_UINT8(VTXDEV_SMARTAUDIO, model.state.vtx.type);

  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(7, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x00, serial.data[0]);
  TEST_ASSERT_EQUAL_UINT8(0x07, serial.data[3]);
  TEST_ASSERT_EQUAL_UINT8(7, serial.data[5]);

  vtx.update(1000 + 300000);
  TEST_ASSERT_EQUAL_size_t(14, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x05, serial.data[10]);
  TEST_ASSERT_EQUAL_UINT8(1, serial.data[12]);
}

void test_vtx_not_started()
{
  Model model;
  VtxTramp tramp(model);
  VtxSmartAudio sa(model);
  tramp.update(1000);
  sa.update(1000);
  TEST_ASSERT_FALSE(model.state.vtx.active);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_vtx_tramp_encode);
  RUN_TEST(test_vtx_tramp_tables);
  RUN_TEST(test_vtx_tramp_update_cycle);
  RUN_TEST(test_vtx_tramp_low_power_disarm);
  RUN_TEST(test_vtx_tramp_tx_full_skips_frame);
  RUN_TEST(test_vtx_smartaudio_encode);
  RUN_TEST(test_vtx_smartaudio_update_cycle);
  RUN_TEST(test_vtx_direct_frequency);
  RUN_TEST(test_vtx_tramp_frequency_mode);
  RUN_TEST(test_vtx_tramp_frequency_unset_skips);
  RUN_TEST(test_vtx_tramp_pit_mode_frequency);
  RUN_TEST(test_vtx_smartaudio_encode_frequency);
  RUN_TEST(test_vtx_smartaudio_frequency_mode);
  RUN_TEST(test_vtx_not_started);
  return UNITY_END();
}
