#include "Connect/Vtx.hpp"
#include "Connect/MspProcessor.hpp"
#include "Connect/VtxSmartAudio.hpp"
#include "Connect/VtxTramp.hpp"
#include "MockSerialStream.hpp"
#include "Utils/Crc.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>
#include "msp/msp_protocol.h"
#include <vector>

using namespace Espfc;
using namespace Espfc::Connect;

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
  MockSerialStream serial;
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
  MockSerialStream serial;
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
  MockSerialStream serial;
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

  VtxSmartAudio::encodePower(f, 0, VTX_SMARTAUDIO_V2);
  TEST_ASSERT_EQUAL_UINT8(0x05, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0, f[4]);
  VtxSmartAudio::encodePower(f, 9, VTX_SMARTAUDIO_V2);
  TEST_ASSERT_EQUAL_UINT8(3, f[4]);
  VtxSmartAudio::encodePower(f, 3, VTX_SMARTAUDIO_V2);
  TEST_ASSERT_EQUAL_UINT8(2, f[4]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 5), f[5]);
}

void test_vtx_smartaudio_encode_power_versions()
{
  uint8_t f[VtxSmartAudio::FRAME_SIZE];
  VtxSmartAudio::encodePower(f, 1, VTX_SMARTAUDIO_V1);
  TEST_ASSERT_EQUAL_UINT8(7, f[4]);
  VtxSmartAudio::encodePower(f, 4, VTX_SMARTAUDIO_V1);
  TEST_ASSERT_EQUAL_UINT8(40, f[4]);
  VtxSmartAudio::encodePower(f, 1, VTX_SMARTAUDIO_V21);
  TEST_ASSERT_EQUAL_UINT8(14 | 0x80, f[4]);
  VtxSmartAudio::encodePower(f, 4, VTX_SMARTAUDIO_V21);
  TEST_ASSERT_EQUAL_UINT8(29 | 0x80, f[4]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 5), f[5]);

  VtxSmartAudio::encodePitModePower(f);
  TEST_ASSERT_EQUAL_UINT8(0x05, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0x80, f[4]);
}

void test_vtx_smartaudio_encode_mode()
{
  uint8_t f[VtxSmartAudio::FRAME_SIZE];
  TEST_ASSERT_EQUAL_size_t(6, VtxSmartAudio::encodeClearPitMode(f));
  TEST_ASSERT_EQUAL_UINT8(0x0B, f[2]);
  TEST_ASSERT_EQUAL_UINT8(0x01, f[3]);
  TEST_ASSERT_EQUAL_UINT8(0x0C, f[4]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 5), f[5]);
}

void test_vtx_smartaudio_encode_pit_mode_frequency()
{
  uint8_t f[VtxSmartAudio::FRAME_SIZE];
  TEST_ASSERT_EQUAL_size_t(7, VtxSmartAudio::encodePitModeFrequency(f, 5584));
  TEST_ASSERT_EQUAL_UINT8(0x09, f[2]);
  TEST_ASSERT_EQUAL_UINT8((5584 >> 8) | 0x80, f[4]);
  TEST_ASSERT_EQUAL_UINT8(5584 & 0xFF, f[5]);
  TEST_ASSERT_EQUAL_UINT8(Utils::crc8_dvb_s2(0, f, 6), f[6]);
}

static void runSmartAudioCycle(Model& model, MockSerialStream& serial)
{
  VtxSmartAudio vtx(model);
  vtx.begin(&serial);
  for (uint32_t i = 0; i < 4; i++)
  {
    vtx.update(1000 + i * 100000);
  }
}

void test_vtx_smartaudio_v2_exits_pit_mode()
{
  Model model;
  MockSerialStream serial;
  model.config.vtx.pitModeFrequency = 5584;
  runSmartAudioCycle(model, serial);
  // channel(7), power(7), mode(7), pit freq(8)
  TEST_ASSERT_EQUAL_size_t(29, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x0B, serial.data[17]);
  TEST_ASSERT_EQUAL_UINT8(0x0C, serial.data[19]);
  TEST_ASSERT_EQUAL_UINT8(0x09, serial.data[24]);
  TEST_ASSERT_EQUAL_UINT8((5584 >> 8) | 0x80, serial.data[26]);
}

void test_vtx_smartaudio_v2_pit_mode_on_skips_clear()
{
  Model model;
  MockSerialStream serial;
  model.config.vtx.pitMode = 1;
  model.config.vtx.pitModeFrequency = 5584;
  runSmartAudioCycle(model, serial);
  // channel(7), power(7), pit freq(8), normal frequency is not replaced by pit frequency
  TEST_ASSERT_EQUAL_size_t(22, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x07, serial.data[3]);
  TEST_ASSERT_EQUAL_UINT8(0x05, serial.data[10]);
  TEST_ASSERT_EQUAL_UINT8(0x09, serial.data[17]);
}

void test_vtx_smartaudio_v21_pit_mode_power()
{
  Model model;
  MockSerialStream serial;
  model.config.vtx.smartAudioVersion = VTX_SMARTAUDIO_V21;
  model.config.vtx.pitMode = 1;
  runSmartAudioCycle(model, serial);
  // channel(7), power 0 dBm(7)
  TEST_ASSERT_EQUAL_size_t(14, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x05, serial.data[10]);
  TEST_ASSERT_EQUAL_UINT8(0x80, serial.data[12]);
}

void test_vtx_smartaudio_v1_tunes_pit_mode_frequency()
{
  Model model;
  MockSerialStream serial;
  model.config.vtx.smartAudioVersion = VTX_SMARTAUDIO_V1;
  model.config.vtx.pitMode = 1;
  model.config.vtx.pitModeFrequency = 5584;
  model.config.vtx.power = 4;
  runSmartAudioCycle(model, serial);
  // frequency(8), power(7), no mode, no pit frequency
  TEST_ASSERT_EQUAL_size_t(15, serial.data.size());
  TEST_ASSERT_EQUAL_UINT8(0x09, serial.data[3]);
  TEST_ASSERT_EQUAL_UINT8(5584 >> 8, serial.data[5]);
  TEST_ASSERT_EQUAL_UINT8(0x05, serial.data[11]);
  TEST_ASSERT_EQUAL_UINT8(40, serial.data[13]);
}

static void processMsp(Model& model, const std::vector<uint8_t>& payload)
{
  Connect::MspProcessor msp(model);
  Connect::MspMessage m;
  m.cmd = MSP_SET_VTX_CONFIG;
  m.append(payload.data(), payload.size());
  Connect::MspResponse r;
  MockSerialStream serial;
  msp.processCommand(m, r, serial);
}

void test_vtx_msp_set_encoded_band_channel()
{
  Model model;
  processMsp(model, {39, 0});
  TEST_ASSERT_EQUAL_UINT8(5, model.config.vtx.band);
  TEST_ASSERT_EQUAL_UINT8(8, model.config.vtx.channel);

  processMsp(model, {40, 0});
  TEST_ASSERT_EQUAL_UINT8(5, model.config.vtx.band);
  TEST_ASSERT_EQUAL_UINT8(8, model.config.vtx.channel);
}

void test_vtx_msp_set_frequency()
{
  Model model;
  processMsp(model, {5800 & 0xFF, 5800 >> 8});
  TEST_ASSERT_EQUAL_UINT8(0, model.config.vtx.band);
  TEST_ASSERT_EQUAL_INT16(5800, model.config.vtx.frequency);

  processMsp(model, {4999 & 0xFF, 4999 >> 8});
  TEST_ASSERT_EQUAL_INT16(5800, model.config.vtx.frequency);
}

void test_vtx_msp_set_pit_mode_frequency()
{
  Model model;
  // freq, power, pit mode, low power disarm, pit freq
  processMsp(model, {39, 0, 2, 1, 0, 5584 & 0xFF, 5584 >> 8});
  TEST_ASSERT_EQUAL_UINT8(1, model.config.vtx.pitMode);
  TEST_ASSERT_EQUAL_INT16(5584, model.config.vtx.pitModeFrequency);

  processMsp(model, {39, 0, 2, 1, 0, 4999 & 0xFF, 4999 >> 8});
  TEST_ASSERT_EQUAL_INT16(5584, model.config.vtx.pitModeFrequency);

  processMsp(model, {39, 0, 2, 0, 0, 0, 0});
  TEST_ASSERT_EQUAL_UINT8(0, model.config.vtx.pitMode);
  TEST_ASSERT_EQUAL_INT16(0, model.config.vtx.pitModeFrequency);
}

void test_vtx_msp_set_extended()
{
  Model model;
  // freq, power, pit mode, low power disarm, pit freq, band, channel, freq
  processMsp(model, {0, 0, 2, 0, 0, 0, 0, 3, 4, 0, 0});
  TEST_ASSERT_EQUAL_UINT8(3, model.config.vtx.band);
  TEST_ASSERT_EQUAL_UINT8(4, model.config.vtx.channel);

  processMsp(model, {0, 0, 2, 0, 0, 0, 0, 6, 1, 0, 0});
  TEST_ASSERT_EQUAL_UINT8(1, model.config.vtx.band);
  TEST_ASSERT_EQUAL_UINT8(1, model.config.vtx.channel);

  processMsp(model, {0, 0, 2, 0, 0, 0, 0, 0, 0, 5900 & 0xFF, 5900 >> 8});
  TEST_ASSERT_EQUAL_UINT8(0, model.config.vtx.band);
  TEST_ASSERT_EQUAL_INT16(5900, model.config.vtx.frequency);
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
  MockSerialStream serial;
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
  MockSerialStream serial;
  VtxTramp vtx(model);
  model.config.vtx.band = 0;
  vtx.begin(&serial);
  vtx.update(1000);
  TEST_ASSERT_EQUAL_size_t(0, serial.data.size());
}

void test_vtx_tramp_pit_mode_frequency()
{
  Model model;
  MockSerialStream serial;
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
  MockSerialStream serial;
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
  MockSerialStream serial;
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

void test_vtx_sanitize_serial_vtx_exclusive()
{
  TEST_ASSERT_EQUAL_INT32(SERIAL_FUNCTION_VTX_TRAMP,
                          Model::sanitizeSerialFunctionMask(SERIAL_FUNCTION_VTX_TRAMP | SERIAL_FUNCTION_TELEMETRY_IBUS |
                                                            SERIAL_FUNCTION_MSP));
  TEST_ASSERT_EQUAL_INT32(SERIAL_FUNCTION_VTX_SMARTAUDIO,
                          Model::sanitizeSerialFunctionMask(SERIAL_FUNCTION_VTX_SMARTAUDIO | SERIAL_FUNCTION_VTX_TRAMP));
  TEST_ASSERT_EQUAL_INT32(SERIAL_FUNCTION_VTX_SMARTAUDIO,
                          Model::sanitizeSerialFunctionMask(SERIAL_FUNCTION_VTX_SMARTAUDIO | SERIAL_FUNCTION_RX_SERIAL));
}

void test_vtx_sanitize_serial_keeps_other_functions()
{
  TEST_ASSERT_EQUAL_INT32(SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_TELEMETRY_FRSKY,
                          Model::sanitizeSerialFunctionMask(SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_TELEMETRY_FRSKY));
  TEST_ASSERT_EQUAL_INT32(SERIAL_FUNCTION_MSP, Model::sanitizeSerialFunctionMask(SERIAL_FUNCTION_MSP |
                                                                                 SERIAL_FUNCTION_TELEMETRY_LTM));
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
  RUN_TEST(test_vtx_smartaudio_encode_power_versions);
  RUN_TEST(test_vtx_smartaudio_encode_mode);
  RUN_TEST(test_vtx_smartaudio_encode_pit_mode_frequency);
  RUN_TEST(test_vtx_smartaudio_v2_exits_pit_mode);
  RUN_TEST(test_vtx_smartaudio_v2_pit_mode_on_skips_clear);
  RUN_TEST(test_vtx_smartaudio_v21_pit_mode_power);
  RUN_TEST(test_vtx_smartaudio_v1_tunes_pit_mode_frequency);
  RUN_TEST(test_vtx_msp_set_encoded_band_channel);
  RUN_TEST(test_vtx_msp_set_frequency);
  RUN_TEST(test_vtx_msp_set_pit_mode_frequency);
  RUN_TEST(test_vtx_msp_set_extended);
  RUN_TEST(test_vtx_direct_frequency);
  RUN_TEST(test_vtx_tramp_frequency_mode);
  RUN_TEST(test_vtx_tramp_frequency_unset_skips);
  RUN_TEST(test_vtx_tramp_pit_mode_frequency);
  RUN_TEST(test_vtx_smartaudio_encode_frequency);
  RUN_TEST(test_vtx_smartaudio_frequency_mode);
  RUN_TEST(test_vtx_sanitize_serial_vtx_exclusive);
  RUN_TEST(test_vtx_sanitize_serial_keeps_other_functions);
  RUN_TEST(test_vtx_not_started);
  return UNITY_END();
}
