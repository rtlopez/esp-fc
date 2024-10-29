#include <unity.h>
#include <EscDriver.h>
#include <Kalman.h>
#include <helper_3dmath.h>
#include "msp/msp_protocol.h"
#include "Math/Utils.h"
#include "Math/Bits.h"
#include "Filter.h"
#include "Control/Pid.h"
#include "Target/QueueAtomic.h"
#include "Utils/RingBuf.h"
#include <printf.h>

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

using namespace Espfc;
using namespace Espfc::Control;

void test_math_map()
{
  TEST_ASSERT_FLOAT_WITHIN(1.f,     0.f, Math::map(   0.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1.f,  1000.f, Math::map( 100.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1.f, -1000.f, Math::map(-100.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1.f,   200.f, Math::map(  20.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));

  TEST_ASSERT_FLOAT_WITHIN(.001f,  0.f, Math::map(   0.0f, -100.0f, 100.0f, -1.0f, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(.001f,  1.f, Math::map( 100.0f, -100.0f, 100.0f, -1.0f, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(.001f, -1.f, Math::map(-100.0f, -100.0f, 100.0f, -1.0f, 1.0f));
}

void test_math_map3()
{
  TEST_ASSERT_FLOAT_WITHIN(1.f,    0.f, Math::map3(  0.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1.f,  500.f, Math::map3( 50.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1.f, -500.f, Math::map3(-50.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
}

void test_math_baro_altitude()
{
  TEST_ASSERT_FLOAT_WITHIN(0.1f,   0.0f, Math::toAltitude(101325.f)); // sea level
  TEST_ASSERT_FLOAT_WITHIN(0.1f,  27.0f, Math::toAltitude(101000.f));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 110.9f, Math::toAltitude(100000.f));
}

void test_math_deadband()
{
  TEST_ASSERT_EQUAL_INT32( 0, Math::deadband(  0, 10));
  TEST_ASSERT_EQUAL_INT32( 0, Math::deadband( 10, 10));
  TEST_ASSERT_EQUAL_INT32( 1, Math::deadband( 11, 10));
  TEST_ASSERT_EQUAL_INT32(-1, Math::deadband(-11, 10));
  TEST_ASSERT_EQUAL_INT32( 0, Math::deadband( -5, 10));
  TEST_ASSERT_EQUAL_INT32(10, Math::deadband( 20, 10));
}

void test_math_bit()
{
  TEST_ASSERT_EQUAL(0, Math::getBit(0, 0));
  TEST_ASSERT_EQUAL(1, Math::getBit(1, 0));
  TEST_ASSERT_EQUAL(0, Math::getBit(1, 1));
  TEST_ASSERT_EQUAL(1, Math::getBit(3, 1));

  TEST_ASSERT_EQUAL_UINT8(0, Math::setBit(0, 0, 0));
  TEST_ASSERT_EQUAL_UINT8(1, Math::setBit(0, 0, 1));
  TEST_ASSERT_EQUAL_UINT8(2, Math::setBit(0, 1, 1));
  TEST_ASSERT_EQUAL_UINT8(3, Math::setBit(2, 0, 1));
}

void test_math_bitmask()
{
  TEST_ASSERT_EQUAL_UINT8(   0, Math::setMasked(0, 1, 0));
  TEST_ASSERT_EQUAL_UINT8(   1, Math::setMasked(0, 1, 1));
  TEST_ASSERT_EQUAL_UINT8(0x38, Math::setMasked(0x00, 0x38, 0xff));
  TEST_ASSERT_EQUAL_UINT8(0xc7, Math::setMasked(0xff, 0x38, 0x00));
}

void test_math_bitmask_lsb()
{
  TEST_ASSERT_EQUAL_UINT8( 1, Math::getMaskLsb(0, 1));
  TEST_ASSERT_EQUAL_UINT8( 2, Math::getMaskLsb(1, 1));
  TEST_ASSERT_EQUAL_UINT8( 4, Math::getMaskLsb(2, 1));
  TEST_ASSERT_EQUAL_UINT8(12, Math::getMaskLsb(2, 2));
  TEST_ASSERT_EQUAL_UINT8(56, Math::getMaskLsb(3, 3));
}

void test_math_bitmask_msb()
{
  TEST_ASSERT_EQUAL_UINT8( 1, Math::getMaskMsb(0, 1));
  TEST_ASSERT_EQUAL_UINT8( 2, Math::getMaskMsb(1, 1));
  TEST_ASSERT_EQUAL_UINT8( 4, Math::getMaskMsb(2, 1));
  TEST_ASSERT_EQUAL_UINT8( 6, Math::getMaskMsb(2, 2));
  TEST_ASSERT_EQUAL_UINT8(14, Math::getMaskMsb(3, 3));
  TEST_ASSERT_EQUAL_UINT8(30, Math::getMaskMsb(4, 4));
}

void test_math_bits_lsb()
{
  TEST_ASSERT_EQUAL_UINT8(0, Math::getBitsLsb(0x00, 1, 1));
  TEST_ASSERT_EQUAL_UINT8(0, Math::getBitsLsb(0x55, 1, 1));
  TEST_ASSERT_EQUAL_UINT8(1, Math::getBitsLsb(0x55, 2, 2));
  TEST_ASSERT_EQUAL_UINT8(1, Math::getBitsLsb(0x55, 4, 2));
  TEST_ASSERT_EQUAL_UINT8(5, Math::getBitsLsb(0x55, 2, 4));

  TEST_ASSERT_EQUAL_UINT8( 8, Math::setBitsLsb(0x00, 3, 4, 1));
  TEST_ASSERT_EQUAL_UINT8(80, Math::setBitsLsb(0x00, 3, 4, 10));
  TEST_ASSERT_EQUAL_UINT8(16, Math::setBitsLsb(0x00, 4, 2, 1));
  TEST_ASSERT_EQUAL_UINT8(160, Math::setBitsLsb(0x00, 4, 4, 10));
}

void test_math_bits_msb()
{
  TEST_ASSERT_EQUAL_UINT8( 0, Math::getBitsMsb(0x00, 1, 1));
  TEST_ASSERT_EQUAL_UINT8( 0, Math::getBitsMsb(0x55, 1, 1));
  TEST_ASSERT_EQUAL_UINT8( 2, Math::getBitsMsb(0x55, 2, 2));
  TEST_ASSERT_EQUAL_UINT8( 2, Math::getBitsMsb(0x55, 4, 2));
  TEST_ASSERT_EQUAL_UINT8(10, Math::getBitsMsb(0x55, 4, 4));

  TEST_ASSERT_EQUAL_UINT8( 1, Math::setBitsMsb(0x00, 3, 4, 1));
  TEST_ASSERT_EQUAL_UINT8(10, Math::setBitsMsb(0x00, 3, 4, 10));
  TEST_ASSERT_EQUAL_UINT8( 8, Math::setBitsMsb(0x00, 6, 4, 1));
  TEST_ASSERT_EQUAL_UINT8(80, Math::setBitsMsb(0x00, 6, 4, 10));
}

void test_math_clock_align()
{
  TEST_ASSERT_EQUAL_INT( 250, Math::alignToClock(1000,  332));
  TEST_ASSERT_EQUAL_INT( 333, Math::alignToClock(1000,  333));
  TEST_ASSERT_EQUAL_INT( 333, Math::alignToClock(1000,  334));
  TEST_ASSERT_EQUAL_INT( 333, Math::alignToClock(1000,  400));
  TEST_ASSERT_EQUAL_INT( 500, Math::alignToClock(1000,  500));
  TEST_ASSERT_EQUAL_INT( 500, Math::alignToClock(1000,  800));
  TEST_ASSERT_EQUAL_INT(1000, Math::alignToClock(1000, 2000));
  TEST_ASSERT_EQUAL_INT( 500, Math::alignToClock(8000,  500));
  TEST_ASSERT_EQUAL_INT( 476, Math::alignToClock(6667,  500));
}

void test_math_peak_detect_full()
{
  using Math::Peak;

  float samples[32] = { 0, 20, 0, 0,  4, 0, 2, 4,  5, 3, 1, 4,  0, 6, 0, 0 };
  Peak peaks[8] = { Peak(), Peak(), Peak(), Peak() };

  Math::peakDetect(samples, 1, 14, 1, peaks, 4);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,  1.f, peaks[0].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.f, peaks[0].value);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 13.f, peaks[1].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  6.f, peaks[1].value);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.92f, peaks[2].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.f, peaks[2].value);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,  4.f, peaks[3].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  4.f, peaks[3].value);
}

void test_math_peak_detect_partial()
{
  using Math::Peak;

  float samples[32] = { 0, 20, 0, 0,  4, 0, 2, 4,  5, 3, 1, 4,  0, 6, 0, 0 };
  Peak peaks[8] = { Peak(), Peak(), Peak(), Peak() };

  Math::peakDetect(samples, 3, 12, 1, peaks, 3);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.92f, peaks[0].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.f, peaks[0].value);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,   4.f, peaks[1].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   4.f, peaks[1].value);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.8f, peaks[2].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   4.f, peaks[2].value);
}

void test_math_peak_sort()
{
  using Math::Peak;

  Peak peaks[8] = { Peak(20, 10), Peak(10, 10), Peak(0, 10), Peak(5, 5) };

  Math::peakSort(peaks, 4);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,  5.f, peaks[0].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.f, peaks[1].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.f, peaks[2].freq);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.f, peaks[3].freq);
}

void test_vector_int16_access()
{
  VectorInt16 v;

  TEST_ASSERT_EQUAL_INT16(0, v.x);
  TEST_ASSERT_EQUAL_INT16(0, v.y);
  TEST_ASSERT_EQUAL_INT16(0, v.z);

  TEST_ASSERT_EQUAL_INT16(0, v[0]);
  TEST_ASSERT_EQUAL_INT16(0, v[1]);
  TEST_ASSERT_EQUAL_INT16(0, v[2]);

  TEST_ASSERT_EQUAL_INT16(0, v.get(0));
  TEST_ASSERT_EQUAL_INT16(0, v.get(1));
  TEST_ASSERT_EQUAL_INT16(0, v.get(2));

  v.set(0, 1);
  v.set(1, 2);
  v.set(2, 3);

  TEST_ASSERT_EQUAL_INT16(1, v.x);
  TEST_ASSERT_EQUAL_INT16(2, v.y);
  TEST_ASSERT_EQUAL_INT16(3, v.z);

  TEST_ASSERT_EQUAL_INT16(1, v[0]);
  TEST_ASSERT_EQUAL_INT16(2, v[1]);
  TEST_ASSERT_EQUAL_INT16(3, v[2]);

  TEST_ASSERT_EQUAL_INT16(1, v.get(0));
  TEST_ASSERT_EQUAL_INT16(2, v.get(1));
  TEST_ASSERT_EQUAL_INT16(3, v.get(2));
}

void test_vector_int16_math()
{
  const VectorInt16 v0(0, -1, 2);

  VectorInt16 v = v0;
  TEST_ASSERT_EQUAL_INT16(0, v.x);
  TEST_ASSERT_EQUAL_INT16(-1, v.y);
  TEST_ASSERT_EQUAL_INT16(2, v.z);

  v += v0;
  TEST_ASSERT_EQUAL_INT16(0, v.x);
  TEST_ASSERT_EQUAL_INT16(-2, v.y);
  TEST_ASSERT_EQUAL_INT16(4, v.z);

  v *= 2;
  TEST_ASSERT_EQUAL_INT16(0, v.x);
  TEST_ASSERT_EQUAL_INT16(-4, v.y);
  TEST_ASSERT_EQUAL_INT16(8, v.z);

  v /= 4;
  TEST_ASSERT_EQUAL_INT16(0, v.x);
  TEST_ASSERT_EQUAL_INT16(-1, v.y);
  TEST_ASSERT_EQUAL_INT16(2, v.z);

  v -= v0;
  TEST_ASSERT_EQUAL_INT16(0, v.x);
  TEST_ASSERT_EQUAL_INT16(0, v.y);
  TEST_ASSERT_EQUAL_INT16(0, v.z);
}

void test_vector_float_math3d()
{
  const VectorFloat v0(0.0f, 1.0f, 0.0f);

  const VectorFloat v1(0.0f, 0.0f, 1.0f);
  const VectorFloat r1 = v0.cross(v1);
  float d1 = v0.dot(v1);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, r1.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r1.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r1.z);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, d1);

  const VectorFloat v2(0.0f, 2.0f, 0.0f);
  const VectorFloat r2 = v0.cross(v2);
  float d2 = v0.dot(v2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r2.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r2.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r2.z);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, d2);

  const VectorFloat v3(2.0f, 2.0f, 0.0f);
  const VectorFloat r3 = v0.cross(v3);
  float d3 = v0.dot(v3);
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, r3.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, r3.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.0f, r3.z);
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  2.0f, d3);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, v0.getMagnitude());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, r3.getMagnitude());

  const VectorFloat n = r3.getNormalized();
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, n.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, n.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, n.z);
}

void test_filter_sanitize_none()
{
  FilterConfig conf(FILTER_NONE, 0);
  TEST_ASSERT_EQUAL(FILTER_NONE, conf.type);
  TEST_ASSERT_EQUAL_INT16(0, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NONE, conf2.type);
  TEST_ASSERT_EQUAL_INT16(0, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_pt1_off()
{
  FilterConfig conf(FILTER_PT1, 0);
  TEST_ASSERT_EQUAL(FILTER_PT1, conf.type);
  TEST_ASSERT_EQUAL_INT16(0, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NONE, conf2.type);
  TEST_ASSERT_EQUAL_INT16(0, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_pt1_nyquist()
{
  FilterConfig conf(FILTER_PT1, 80);
  TEST_ASSERT_EQUAL(FILTER_PT1, conf.type);
  TEST_ASSERT_EQUAL_INT16(80, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_PT1, conf2.type);
  TEST_ASSERT_EQUAL_INT16(49, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_pt1()
{
  FilterConfig conf(FILTER_PT1, 20);
  TEST_ASSERT_EQUAL(FILTER_PT1, conf.type);
  TEST_ASSERT_EQUAL_INT16(20, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_PT1, conf2.type);
  TEST_ASSERT_EQUAL_INT16(20, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_biquad_lpf()
{
  FilterConfig conf(FILTER_BIQUAD, 20);
  TEST_ASSERT_EQUAL(FILTER_BIQUAD, conf.type);
  TEST_ASSERT_EQUAL_INT16(20, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_BIQUAD, conf2.type);
  TEST_ASSERT_EQUAL_INT16(20, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_biquad_lpf_off()
{
  FilterConfig conf(FILTER_BIQUAD, 0);
  TEST_ASSERT_EQUAL(FILTER_BIQUAD, conf.type);
  TEST_ASSERT_EQUAL_INT16(0, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NONE, conf2.type);
  TEST_ASSERT_EQUAL_INT16(0, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_biquad_lpf_nyqist()
{
  FilterConfig conf(FILTER_BIQUAD, 80);
  TEST_ASSERT_EQUAL(FILTER_BIQUAD, conf.type);
  TEST_ASSERT_EQUAL_INT16(80, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_BIQUAD, conf2.type);
  TEST_ASSERT_EQUAL_INT16(49, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_biquad_notch()
{
  FilterConfig conf(FILTER_NOTCH, 40, 30);
  TEST_ASSERT_EQUAL(FILTER_NOTCH, conf.type);
  TEST_ASSERT_EQUAL_INT16(40, conf.freq);
  TEST_ASSERT_EQUAL_INT16(30, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NOTCH, conf2.type);
  TEST_ASSERT_EQUAL_INT16(40, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(30, conf2.cutoff);
}

void test_filter_sanitize_biquad_notch_off_freq()
{
  FilterConfig conf(FILTER_NOTCH, 0, 30);
  TEST_ASSERT_EQUAL(FILTER_NOTCH, conf.type);
  TEST_ASSERT_EQUAL_INT16(0, conf.freq);
  TEST_ASSERT_EQUAL_INT16(30, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NONE, conf2.type);
  TEST_ASSERT_EQUAL_INT16(0, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_biquad_notch_off_cut()
{
  FilterConfig conf(FILTER_NOTCH, 40, 0);
  TEST_ASSERT_EQUAL(FILTER_NOTCH, conf.type);
  TEST_ASSERT_EQUAL_INT16(40, conf.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NONE, conf2.type);
  TEST_ASSERT_EQUAL_INT16(40, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(0, conf2.cutoff);
}

void test_filter_sanitize_biquad_notch_nyquist()
{
  FilterConfig conf(FILTER_NOTCH, 80, 70);
  TEST_ASSERT_EQUAL(FILTER_NOTCH, conf.type);
  TEST_ASSERT_EQUAL_INT16(80, conf.freq);
  TEST_ASSERT_EQUAL_INT16(70, conf.cutoff);

  FilterConfig conf2 = conf.sanitize(100);
  TEST_ASSERT_EQUAL(FILTER_NOTCH, conf2.type);
  TEST_ASSERT_EQUAL_INT16(49, conf2.freq);
  TEST_ASSERT_EQUAL_INT16(48, conf2.cutoff);
}

void assert_filter_off(Filter& filter)
{
  TEST_ASSERT_EQUAL(FILTER_NONE, filter._conf.type);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, filter.update(0.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, filter.update(0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, filter.update(1.5f));
}

void test_filter_default()
{
  Filter filter;
  assert_filter_off(filter);
}

void test_filter_none()
{
  Filter filter;
  const FilterConfig config(FILTER_NONE, 0);
  filter.begin(config, 100);
  assert_filter_off(filter);
}

void test_filter_pt1_off()
{
  Filter filter;
  const FilterConfig config(FILTER_PT1, 0);
  filter.begin(config, 100);
  assert_filter_off(filter);
}

void test_filter_pt1_50_100()
{
  Filter filter;
  const FilterConfig config(FILTER_PT1, 50);
  filter.begin(config, 100);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.076f, filter.update(0.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.395f, filter.update(0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.229f, filter.update(1.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.434f, filter.update(1.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.484f, filter.update(1.5f));
}

void test_filter_pt1_10_100_step()
{
  Filter filter;
  const FilterConfig config(FILTER_PT1, 10);
  filter.begin(config, 100);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.386f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.623f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.768f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.858f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.913f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.946f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.967f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.980f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.988f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.992f, filter.update(1.0f));
}

void test_filter_pt1_above_nyquist()
{
  Filter filter;
  const FilterConfig config(FILTER_NOTCH, 200);
  filter.begin(config, 100);
  TEST_ASSERT_EQUAL_INT(100, filter._rate);
  TEST_ASSERT_LESS_OR_EQUAL_INT(50, filter._conf.freq);
  TEST_ASSERT_GREATER_THAN(10, filter._conf.freq);
}

void test_filter_pt2_10_100_step()
{
  Filter filter;
  const FilterConfig config(FILTER_PT2, 10);
  filter.begin(config, 100);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.244f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.490f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.678f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.804f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.885f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.933f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.962f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.978f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.988f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.993f, filter.update(1.0f));
}

void test_filter_pt3_10_100_step()
{
  Filter filter;
  const FilterConfig config(FILTER_PT3, 10);
  filter.begin(config, 100);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.168f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.394f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.596f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.748f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.850f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.913f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.952f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.973f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.986f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.992f, filter.update(1.0f));
}

void test_filter_biquad_off()
{
  Filter filter;
  const FilterConfig config(FILTER_BIQUAD, 0);
  filter.begin(config, 100);
  assert_filter_off(filter);
}

void test_filter_biquad_20_100()
{
  Filter filter;
  const FilterConfig config(FILTER_BIQUAD, 20);
  filter.begin(config, 100);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.020f, filter.update(0.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.152f, filter.update(0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.589f, filter.update(1.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.220f, filter.update(1.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.575f, filter.update(1.5f));
}

void test_filter_biquad_10_100_step()
{
  Filter filter;
  const FilterConfig config(FILTER_BIQUAD, 10);
  filter.begin(config, 100);

  TEST_ASSERT_EQUAL(FILTER_BIQUAD, filter._conf.type);
  TEST_ASSERT_EQUAL(100, filter._rate);
  TEST_ASSERT_EQUAL(10, filter._conf.freq);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.067f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.279f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.561f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.796f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.948f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.025f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.050f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.047f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.033f, filter.update(1.0f));
}

void test_filter_notch_df1_150_200_1000()
{
  Filter filter;
  const FilterConfig config(FILTER_NOTCH_DF1, 200, 150);
  filter.begin(config, 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.783f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.678f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.967f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.166f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.099f, filter.update(1.0f));
}

void test_filter_notch_df1_150_200_1000_reconf()
{
  Filter filter;
  const FilterConfig config(FILTER_NOTCH_DF1, 200, 150);
  filter.begin(config, 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.783f, filter.update(1.0f));
  filter.reconfigure(200, 150);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.678f, filter.update(1.0f));
  filter.reconfigure(200, 150);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.967f, filter.update(1.0f));
  filter.reconfigure(200, 150);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.166f, filter.update(1.0f));
  filter.reconfigure(200, 150);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.099f, filter.update(1.0f));
}

void test_filter_notch_q()
{
  Filter filter;

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.667f, filter.getNotchQ(200, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.667f, filter.getNotchQApprox(200, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.007f, filter.getNotchQ(200, 124));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.007f, filter.getNotchQApprox(200, 124));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.714f, filter.getNotchQ(200, 150));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.714f, filter.getNotchQApprox(200, 150));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.744f, filter.getNotchQ(200, 190));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.744f, filter.getNotchQApprox(200, 190));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.737f, filter.getNotchQ(100, 90));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.737f, filter.getNotchQApprox(100, 90));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.222f, filter.getNotchQ(100, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.222f, filter.getNotchQApprox(100, 80));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.216f, filter.getNotchQ(100, 67));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.216f, filter.getNotchQApprox(100, 67));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.667f, filter.getNotchQ(300, 150));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.667f, filter.getNotchQApprox(300, 150));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.884f, filter.getNotchQ(300, 175));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.884f, filter.getNotchQApprox(300, 175));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.200f, filter.getNotchQ(300, 200));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.200f, filter.getNotchQApprox(300, 200));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.733f, filter.getNotchQ(400, 350));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.733f, filter.getNotchQApprox(400, 350));
}

void test_filter_notch_no_cutoff()
{
  Filter filter;
  const FilterConfig config(FILTER_NOTCH, 50, 0);
  filter.begin(config, 100);
  assert_filter_off(filter);
}

void test_filter_notch_above_nyquist()
{
  Filter filter;
  const FilterConfig config(FILTER_NOTCH, 200, 150);
  filter.begin(config, 100);
  TEST_ASSERT_EQUAL_INT(100, filter._rate);
  TEST_ASSERT_LESS_OR_EQUAL_INT(100, filter._conf.freq);
  TEST_ASSERT_LESS_THAN(filter._conf.freq, filter._conf.cutoff);
}

void test_filter_fir2_off()
{
  Filter filter;
  const FilterConfig config(FILTER_FIR2, 0);
  filter.begin(config, 100);
  TEST_ASSERT_EQUAL_INT(100, filter._rate);
  TEST_ASSERT_LESS_OR_EQUAL_INT(0, filter._conf.freq);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, filter.update(1.0f));
}

void test_filter_fir2_on()
{
  Filter filter;
  const FilterConfig config(FILTER_FIR2, 1);
  filter.begin(config, 100);
  TEST_ASSERT_EQUAL_INT(100, filter._rate);
  TEST_ASSERT_LESS_OR_EQUAL_INT(1, filter._conf.freq);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.500f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, filter.update(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, filter.update(1.0f));
}

void test_pid_init()
{
  Pid pid;
  pid.rate = 100;
  pid.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, pid.rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.01f, pid.dt);

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, pid.Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.Kf);

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.3f, pid.iLimit);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.oLimit);

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.pScale);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.iScale);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.dScale);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.fScale);

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.iTermError);

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.fTerm);

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.prevMeasurement);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.prevError);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.prevSetpoint);

  TEST_ASSERT_FALSE(pid.outputSaturated);

  TEST_ASSERT_EQUAL(0, pid.itermRelax);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.itermRelaxBase);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.itermRelaxFactor);
}

void ensure(Pid& pid, float rate = 100.0f)
{
  pid.rate = rate;
}

void gain(Pid& pid, float p, float i, float d, float f)
{
  pid.Kp = p;
  pid.Ki = i;
  pid.Kd = d;
  pid.Kf = f;
}

void test_pid_update_p()
{
  Pid pid;
  ensure(pid);
  gain(pid, 1, 0, 0, 0);
  pid.begin();

  float result = pid.update(0.1f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, result);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.prevSetpoint);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
}

void test_pid_update_i()
{
  Pid pid;
  ensure(pid);
  gain(pid, 0, 10, 0, 0);
  pid.begin();

  float result = pid.update(0.2f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.02f, result);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.prevSetpoint);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.prevError);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasurement);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.02f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);

  float result2 = pid.update(0.2f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.04f, result2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.prevSetpoint);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.prevError);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasurement);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.04f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
}

void test_pid_update_i_limit()
{
  Pid pid;
  ensure(pid);
  gain(pid, 0, 10, 0, 0);
  pid.iLimit = 0.2f;
  pid.begin();

  float result1 = pid.update(0.8f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.08f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.08f, result1);

  float result2 = pid.update(0.8f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.16f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.16f, result2);

  float result3 = pid.update(0.8f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.20f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.20f, result3);
}

void test_pid_update_i_relax()
{
  Pid pid;
  ensure(pid);
  gain(pid, 0, 10, 0, 0);
  pid.itermRelax = ITERM_RELAX_RP;
  pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, 15), pid.rate);
  pid.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, pid.iScale);

  float result0 = pid.update(0.0f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result0);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTermError);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.itermRelaxBase);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pid.itermRelaxFactor);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);

  float result1 = pid.update(1.f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.262f, pid.iTermError);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.515f, pid.itermRelaxBase);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.262f, pid.itermRelaxFactor);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.026f, result1);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.026f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, pid.fTerm);

  float result2 = pid.update(3.f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.026f, result2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.000f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, pid.iTermError);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.294f, pid.itermRelaxBase);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, pid.itermRelaxFactor);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.026f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.00f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.00f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.00f, pid.fTerm);
}

void test_pid_update_d()
{
  Pid pid;
  ensure(pid);
  gain(pid, 0, 0, 0.1f, 0);
  pid.begin();

  float result1 = pid.update(0.0f, -0.05f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.05f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.05f, pid.prevMeasurement);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, result1);

  float result2 = pid.update(0.0f, 0.0f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasurement);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, result2);
}

void test_pid_update_f()
{
  Pid pid;
  ensure(pid);
  gain(pid, 0, 0, 0, 0.1f);
  pid.begin();

  float result1 = pid.update(-0.05f, 0.0f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.05f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.05f, pid.prevSetpoint);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, result1);

  float result2 = pid.update(0.0f, 0.0f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.error);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasurement);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, result2);
}

void test_pid_update_sum()
{
  Pid pid;
  ensure(pid);
  gain(pid, 1, 100, 0.1f, 0.01f);
  pid.begin();

  float result = pid.update(0.1f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.error);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, result);
}

void test_pid_update_sum_limit()
{
  Pid pid;
  ensure(pid);
  gain(pid, 1, 100, 0.01f, 0.01f);
  pid.begin();

  float result = pid.update(0.5f, 0.f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.error);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.pTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, pid.iTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.fTerm);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, result);
}

void test_queue_atomic()
{
  QueueAtomic<int, 3> q;
  int e1 =  1, e2 =  2, e3 =  3, e4 =  4;
  int r1 = 91, r2 = 92, r3 = 93, r4 = 94;

  // empty
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());

  TEST_ASSERT_FALSE(q.pop(r1));
  TEST_ASSERT_EQUAL(91, r1);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());

  // push first element
  TEST_ASSERT_TRUE(q.push(e1));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());

  // pop last element
  TEST_ASSERT_TRUE(q.pop(r1));
  TEST_ASSERT_EQUAL(1, r1);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());

  // pop element again
  TEST_ASSERT_FALSE(q.pop(r1));
  TEST_ASSERT_EQUAL(1, r1);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());

  // make full
  TEST_ASSERT_TRUE(q.push(e1));
  TEST_ASSERT_TRUE(q.push(e2));
  TEST_ASSERT_TRUE(q.push(e3));
  TEST_ASSERT_FALSE(q.push(e4));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_TRUE(q.isFull());

  // make empty
  TEST_ASSERT_TRUE(q.pop(r1));
  TEST_ASSERT_EQUAL(1, r1);

  TEST_ASSERT_TRUE(q.pop(r2));
  TEST_ASSERT_EQUAL(2, r2);

  TEST_ASSERT_TRUE(q.pop(r3));
  TEST_ASSERT_EQUAL(3, r3);

  TEST_ASSERT_FALSE(q.pop(r4));
  TEST_ASSERT_EQUAL(94, r4);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
}

void test_ring_buf()
{
  Utils::RingBuf<uint8_t, 3> q;
  uint8_t e1 =  1, e2 =  2, e3 =  3, e4 =  4;
  uint8_t r1 = 91, r2 = 92, r3 = 93, r4 = 94;

  // empty
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(0, q.size());
  TEST_ASSERT_EQUAL(3, q.available());

  TEST_ASSERT_FALSE(q.pop(r1));
  TEST_ASSERT_EQUAL(91, r1);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(0, q.size());
  TEST_ASSERT_EQUAL(3, q.available());

  // push first element
  TEST_ASSERT_TRUE(q.push(e1));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(1, q.size());
  TEST_ASSERT_EQUAL(2, q.available());

  // pop last element
  TEST_ASSERT_TRUE(q.pop(r1));
  TEST_ASSERT_EQUAL(1, r1);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(0, q.size());
  TEST_ASSERT_EQUAL(3, q.available());

  // pop element again
  TEST_ASSERT_FALSE(q.pop(r1));
  TEST_ASSERT_EQUAL(1, r1);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(0, q.size());
  TEST_ASSERT_EQUAL(3, q.available());

  // make full
  TEST_ASSERT_TRUE(q.push(e1));
  TEST_ASSERT_TRUE(q.push(e2));
  TEST_ASSERT_TRUE(q.push(e3));
  TEST_ASSERT_FALSE(q.push(e4));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_TRUE(q.isFull());
  TEST_ASSERT_EQUAL(3, q.size());
  TEST_ASSERT_EQUAL(0, q.available());

  // make empty
  TEST_ASSERT_TRUE(q.pop(r1));

  TEST_ASSERT_EQUAL(1, r1);
  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(2, q.size());
  TEST_ASSERT_EQUAL(1, q.available());

  TEST_ASSERT_TRUE(q.pop(r2));

  TEST_ASSERT_EQUAL(2, r2);
  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(1, q.size());
  TEST_ASSERT_EQUAL(2, q.available());

  TEST_ASSERT_TRUE(q.pop(r3));

  TEST_ASSERT_EQUAL(3, r3);
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL(0, q.size());
  TEST_ASSERT_EQUAL(3, q.available());

  TEST_ASSERT_FALSE(q.pop(r4));

  TEST_ASSERT_EQUAL(94, r4);
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(0, q.size());
  TEST_ASSERT_EQUAL_UINT32(3, q.available());

  // push 2 elements at once
  uint8_t a1[3] = {11, 22, 33};
  uint8_t a2[3] = {10, 20, 30};
  TEST_ASSERT_EQUAL_UINT32(2, q.push(a1, 2));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(2, q.size());
  TEST_ASSERT_EQUAL_UINT32(1, q.available());

  // try to pop 3 elements at once
  TEST_ASSERT_EQUAL_UINT32(2, q.pop(a2, 3));
  TEST_ASSERT_EQUAL_UINT8(11, a2[0]);
  TEST_ASSERT_EQUAL_UINT8(22, a2[1]);
  TEST_ASSERT_EQUAL_UINT8(30, a2[2]);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(0, q.size());
  TEST_ASSERT_EQUAL_UINT32(3, q.available());
}

void test_ring_buf2()
{
  Utils::RingBuf<int, 8> q;
  int x[] = {
      100, 101, 102, 103, 104, 105, 106, 107,
      108, 109, 110, 111, 112, 113, 114, 115,
  };
  int y[] = {
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
  };

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(0, q.size());
  TEST_ASSERT_EQUAL_UINT32(8, q.available());

  // push 6
  TEST_ASSERT_EQUAL_UINT32(6, q.push(&x[0], 6));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(6, q.size());
  TEST_ASSERT_EQUAL_UINT32(2, q.available());

  // pop 4
  TEST_ASSERT_EQUAL_UINT32(4, q.pop(&y[0], 4));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(2, q.size());
  TEST_ASSERT_EQUAL_UINT32(6, q.available());

  TEST_ASSERT_EQUAL_INT32(100, y[0]);
  TEST_ASSERT_EQUAL_INT32(101, y[1]);
  TEST_ASSERT_EQUAL_INT32(102, y[2]);
  TEST_ASSERT_EQUAL_INT32(103, y[3]);

  // push 6
  TEST_ASSERT_EQUAL_UINT32(6, q.push(&x[6], 6));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_TRUE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(8, q.size());
  TEST_ASSERT_EQUAL_UINT32(0, q.available());

  // pop 4
  TEST_ASSERT_EQUAL_UINT32(4, q.pop(&y[0], 4));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_UINT32(4, q.size());
  TEST_ASSERT_EQUAL_UINT32(4, q.available());

  TEST_ASSERT_EQUAL_INT32(104, y[0]);
  TEST_ASSERT_EQUAL_INT32(105, y[1]);
  TEST_ASSERT_EQUAL_INT32(106, y[2]);
  TEST_ASSERT_EQUAL_INT32(107, y[3]);
}

void test_align_addr_to_write()
{
  TEST_ASSERT_EQUAL_UINT32(  0, Math::alignAddressToWrite(  0,  8, 16));
  TEST_ASSERT_EQUAL_UINT32( 16, Math::alignAddressToWrite(  0, 16, 16));
  TEST_ASSERT_EQUAL_UINT32( 16, Math::alignAddressToWrite(  0, 24, 16));
  TEST_ASSERT_EQUAL_UINT32(144, Math::alignAddressToWrite(128, 16, 16));
  TEST_ASSERT_EQUAL_UINT32( 32, Math::alignAddressToWrite(  0, 32, 16));
  TEST_ASSERT_EQUAL_UINT32(128, Math::alignAddressToWrite(100, 32, 16));
}

void test_rotation_matrix_no_rotation()
{
  VectorFloat v{1.f, 2.f, 3.f};
  RotationMatrixFloat rm;

  VectorFloat r = rm.apply(v);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.f, r.z);
}

void test_rotation_matrix_90_roll()
{
  VectorFloat v{0.f, 0.f, 1.f};
  RotationMatrixFloat rm;
  rm.init(VectorFloat{
    Math::toRad(90),
    Math::toRad(0),
    Math::toRad(0),
  });

  VectorFloat r = rm.apply(v);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  1.f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.f, r.z);
}

void test_rotation_matrix_90_pitch()
{
  VectorFloat v{0.f, 0.f, 1.f};
  RotationMatrixFloat rm;
  rm.init(VectorFloat{
    Math::toRad(0),
    Math::toRad(90),
    Math::toRad(0),
  });

  VectorFloat r = rm.apply(v);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.f, r.z);
}

void test_rotation_matrix_90_yaw()
{
  VectorFloat v{1.f, 2.f, 3.f};
  RotationMatrixFloat rm;
  rm.init(VectorFloat{
    Math::toRad(0),
    Math::toRad(0),
    Math::toRad(90),
  });

  VectorFloat r = rm.apply(v);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,  2.f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  3.f, r.z);
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();

  RUN_TEST(test_math_map);
  RUN_TEST(test_math_map3);
  RUN_TEST(test_math_deadband);

  RUN_TEST(test_math_bit);
  RUN_TEST(test_math_bitmask);
  RUN_TEST(test_math_bitmask_lsb);
  RUN_TEST(test_math_bitmask_msb);
  RUN_TEST(test_math_bits_lsb);
  RUN_TEST(test_math_bits_msb);

  RUN_TEST(test_math_clock_align);

  RUN_TEST(test_math_baro_altitude);
  RUN_TEST(test_math_peak_detect_full);
  RUN_TEST(test_math_peak_detect_partial);
  RUN_TEST(test_math_peak_sort);

  RUN_TEST(test_vector_int16_access);
  RUN_TEST(test_vector_int16_math);
  RUN_TEST(test_vector_float_math3d);

  RUN_TEST(test_filter_sanitize_none);
  RUN_TEST(test_filter_sanitize_pt1);
  RUN_TEST(test_filter_sanitize_pt1_off);
  RUN_TEST(test_filter_sanitize_pt1_nyquist);
  RUN_TEST(test_filter_sanitize_biquad_lpf);
  RUN_TEST(test_filter_sanitize_biquad_lpf_off);
  RUN_TEST(test_filter_sanitize_biquad_lpf_nyqist);
  RUN_TEST(test_filter_sanitize_biquad_notch);
  RUN_TEST(test_filter_sanitize_biquad_notch_off_freq);
  RUN_TEST(test_filter_sanitize_biquad_notch_off_cut);
  RUN_TEST(test_filter_sanitize_biquad_notch_nyquist);

  RUN_TEST(test_filter_default);
  RUN_TEST(test_filter_none);
  RUN_TEST(test_filter_pt1_off);
  RUN_TEST(test_filter_pt1_50_100);
  RUN_TEST(test_filter_pt1_10_100_step);
  RUN_TEST(test_filter_pt1_above_nyquist);
  RUN_TEST(test_filter_pt2_10_100_step);
  RUN_TEST(test_filter_pt3_10_100_step);
  RUN_TEST(test_filter_biquad_off);
  RUN_TEST(test_filter_biquad_20_100);
  RUN_TEST(test_filter_biquad_10_100_step);
  RUN_TEST(test_filter_notch_df1_150_200_1000);
  RUN_TEST(test_filter_notch_df1_150_200_1000_reconf);
  RUN_TEST(test_filter_notch_q);
  RUN_TEST(test_filter_notch_no_cutoff);
  RUN_TEST(test_filter_notch_above_nyquist);
  RUN_TEST(test_filter_fir2_off);
  RUN_TEST(test_filter_fir2_on);

  RUN_TEST(test_pid_init);
  RUN_TEST(test_pid_update_p);
  RUN_TEST(test_pid_update_i);
  RUN_TEST(test_pid_update_i_limit);
  RUN_TEST(test_pid_update_i_relax);
  RUN_TEST(test_pid_update_d);
  RUN_TEST(test_pid_update_f);
  RUN_TEST(test_pid_update_sum);
  RUN_TEST(test_pid_update_sum_limit);

  RUN_TEST(test_queue_atomic);
  RUN_TEST(test_ring_buf);
  RUN_TEST(test_ring_buf2);
  RUN_TEST(test_align_addr_to_write);

  RUN_TEST(test_rotation_matrix_no_rotation);
  RUN_TEST(test_rotation_matrix_90_roll);
  RUN_TEST(test_rotation_matrix_90_pitch);
  RUN_TEST(test_rotation_matrix_90_yaw);

  return UNITY_END();
}