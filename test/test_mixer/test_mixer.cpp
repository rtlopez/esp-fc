#include <Output/Mixers.hpp>
#include <unity.h>

using namespace Espfc;
using Espfc::Output::Mixers;

namespace {

const MixerConfig kCustom(2, nullptr);

size_t countEntries(const MixerConfig& cfg)
{
  size_t n = 0;
  while (cfg.mixes[n].src != MIXER_SOURCE_NULL)
  {
    n++;
  }
  return n;
}

void assertEntry(const MixerConfig& cfg, size_t idx, int8_t src, int8_t dst, int16_t rate)
{
  TEST_ASSERT_EQUAL_INT8(src, cfg.mixes[idx].src);
  TEST_ASSERT_EQUAL_INT8(dst, cfg.mixes[idx].dst);
  TEST_ASSERT_EQUAL_INT16(rate, cfg.mixes[idx].rate);
}

} // namespace

void test_mixer_quadx()
{
  const MixerConfig m = Mixers::getMixer(FC_MIXER_QUADX, kCustom);
  TEST_ASSERT_EQUAL_UINT8(4, m.count);
  TEST_ASSERT_NOT_NULL(m.mixes);
  TEST_ASSERT_EQUAL_size_t(16, countEntries(m));
  assertEntry(m, 0, MIXER_SOURCE_ROLL, 0, -100);
  assertEntry(m, 3, MIXER_SOURCE_ROLL, 3, 100);
  assertEntry(m, 4, MIXER_SOURCE_PITCH, 0, 100);
  assertEntry(m, 8, MIXER_SOURCE_YAW, 0, -100);
  assertEntry(m, 15, MIXER_SOURCE_THRUST, 3, 100);
}

void test_mixer_quadx_1234()
{
  const MixerConfig m = Mixers::getMixer(FC_MIXER_QUADX_1234, kCustom);
  TEST_ASSERT_EQUAL_UINT8(4, m.count);
  TEST_ASSERT_EQUAL_size_t(16, countEntries(m));
  assertEntry(m, 0, MIXER_SOURCE_ROLL, 0, 100);
  assertEntry(m, 1, MIXER_SOURCE_ROLL, 1, -100);
  assertEntry(m, 6, MIXER_SOURCE_PITCH, 2, 100);
  assertEntry(m, 15, MIXER_SOURCE_THRUST, 3, 100);
}

void test_mixer_quadx_variants_differ()
{
  const MixerConfig a = Mixers::getMixer(FC_MIXER_QUADX, kCustom);
  const MixerConfig b = Mixers::getMixer(FC_MIXER_QUADX_1234, kCustom);
  TEST_ASSERT_TRUE(a.mixes != b.mixes);
  TEST_ASSERT_NOT_EQUAL(a.mixes[0].rate, b.mixes[0].rate);
}

void test_mixer_tricopter()
{
  const MixerConfig m = Mixers::getMixer(FC_MIXER_TRI, kCustom);
  TEST_ASSERT_EQUAL_UINT8(4, m.count);
  TEST_ASSERT_EQUAL_size_t(16, countEntries(m));
  assertEntry(m, 3, MIXER_SOURCE_ROLL, 3, 0);
  assertEntry(m, 4, MIXER_SOURCE_PITCH, 0, 133);
  assertEntry(m, 11, MIXER_SOURCE_YAW, 3, 100);
  assertEntry(m, 15, MIXER_SOURCE_THRUST, 3, 0);
}

void test_mixer_gimbal()
{
  const MixerConfig m = Mixers::getMixer(FC_MIXER_GIMBAL, kCustom);
  TEST_ASSERT_EQUAL_UINT8(2, m.count);
  TEST_ASSERT_EQUAL_size_t(4, countEntries(m));
  assertEntry(m, 0, MIXER_SOURCE_PITCH, 0, 100);
  assertEntry(m, 1, MIXER_SOURCE_PITCH, 1, 100);
  assertEntry(m, 2, MIXER_SOURCE_YAW, 0, 100);
  assertEntry(m, 3, MIXER_SOURCE_YAW, 1, -100);
}

void test_mixer_custom_returns_custom_config()
{
  MixerEntry entries[] = {{MIXER_SOURCE_ROLL, 0, 50}, {}};
  const MixerConfig custom(2, entries);

  const MixerType types[] = {FC_MIXER_CUSTOM, FC_MIXER_CUSTOM_TRI, FC_MIXER_CUSTOM_AIRPLANE};
  for (MixerType t : types)
  {
    const MixerConfig m = Mixers::getMixer(t, custom);
    TEST_ASSERT_EQUAL_UINT8(2, m.count);
    TEST_ASSERT_TRUE(m.mixes == entries);
  }
}

void test_mixer_unsupported_returns_empty()
{
  const MixerType types[] = {FC_MIXER_QUADP, FC_MIXER_HEX6, FC_MIXER_AIRPLANE, FC_MIXER_PPM_TO_SERVO};
  for (MixerType t : types)
  {
    const MixerConfig m = Mixers::getMixer(t, kCustom);
    TEST_ASSERT_EQUAL_UINT8(0, m.count);
    TEST_ASSERT_NOT_NULL(m.mixes);
    TEST_ASSERT_EQUAL_size_t(0, countEntries(m));
  }
}

void test_mixer_invalid_type_returns_empty()
{
  const MixerConfig m = Mixers::getMixer(static_cast<MixerType>(0), kCustom);
  TEST_ASSERT_EQUAL_UINT8(0, m.count);
  TEST_ASSERT_NOT_NULL(m.mixes);
  TEST_ASSERT_EQUAL_size_t(0, countEntries(m));

  const MixerConfig m2 = Mixers::getMixer(static_cast<MixerType>(100), kCustom);
  TEST_ASSERT_EQUAL_UINT8(0, m2.count);
  TEST_ASSERT_NOT_NULL(m2.mixes);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_mixer_quadx);
  RUN_TEST(test_mixer_quadx_1234);
  RUN_TEST(test_mixer_quadx_variants_differ);
  RUN_TEST(test_mixer_tricopter);
  RUN_TEST(test_mixer_gimbal);
  RUN_TEST(test_mixer_custom_returns_custom_config);
  RUN_TEST(test_mixer_unsupported_returns_empty);
  RUN_TEST(test_mixer_invalid_type_returns_empty);
  return UNITY_END();
}
