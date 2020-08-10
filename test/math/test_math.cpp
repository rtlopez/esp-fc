#include <unity.h>
#include "Math/Utils.h"
#include "helper_3dmath.h"
#include "Filter.h"
#include "Pid.h"

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

using namespace Espfc;

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

void test_math_deadband()
{
    TEST_ASSERT_EQUAL_INT32( 0, Math::deadband(  0, 10));
    TEST_ASSERT_EQUAL_INT32( 0, Math::deadband( 10, 10));
    TEST_ASSERT_EQUAL_INT32( 1, Math::deadband( 11, 10));
    TEST_ASSERT_EQUAL_INT32(-1, Math::deadband(-11, 10));
    TEST_ASSERT_EQUAL_INT32( 0, Math::deadband( -5, 10));
    TEST_ASSERT_EQUAL_INT32(10, Math::deadband( 20, 10));
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
    TEST_ASSERT_EQUAL_INT16(50, conf2.freq);
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
    TEST_ASSERT_EQUAL_INT16(50, conf2.freq);
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
    TEST_ASSERT_EQUAL_INT16(50, conf2.freq);
    TEST_ASSERT_EQUAL_INT16(49, conf2.cutoff);
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
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.397f, filter.update(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.234f, filter.update(1.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.436f, filter.update(1.5f));
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
}

void ensure(Pid& pid, float rate = 100.0f)
{
    pid.rate = rate;
    pid.pScale = 1.0f;
    pid.iScale = 1.0f;
    pid.dScale = 1.0f;
    pid.fScale = 1.0f;
    pid.oLimit = 1.0f;
    pid.iLimit = 0.2f;
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
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasure);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.02f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
}

void test_pid_update_i_limit()
{
    Pid pid;
    ensure(pid);
    gain(pid, 0, 10, 0, 0);
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

void test_pid_update_d()
{
    Pid pid;
    ensure(pid);
    gain(pid, 0, 0, 0.1f, 0);
    pid.begin();

    float result1 = pid.update(0.0f, -0.05f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.05f, pid.error);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.05f, pid.prevMeasure);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.fTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, result1);

    float result2 = pid.update(0.0f, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.error);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasure);
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
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasure);
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
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.fTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, result);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_math_map);
    RUN_TEST(test_math_map3);
    RUN_TEST(test_math_deadband);

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
    RUN_TEST(test_pid_update_d);
    RUN_TEST(test_pid_update_f);
    RUN_TEST(test_pid_update_sum);
    RUN_TEST(test_pid_update_sum_limit);

    UNITY_END();

    return 0;
}