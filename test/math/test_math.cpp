#include <unity.h>
#include "MathUtil.h"
#include "Filter.h"
#include "Pid.h"

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_math_map()
{
    TEST_ASSERT_FLOAT_WITHIN(1.f,     0.f, Espfc::Math::map(   0.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f,  1000.f, Espfc::Math::map( 100.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f, -1000.f, Espfc::Math::map(-100.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f,   200.f, Espfc::Math::map(  20.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));

    TEST_ASSERT_FLOAT_WITHIN(.001f,  0.f, Espfc::Math::map(   0.0f, -100.0f, 100.0f, -1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(.001f,  1.f, Espfc::Math::map( 100.0f, -100.0f, 100.0f, -1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(.001f, -1.f, Espfc::Math::map(-100.0f, -100.0f, 100.0f, -1.0f, 1.0f));
}

void test_math_map3()
{
    TEST_ASSERT_FLOAT_WITHIN(1.f,    0.f, Espfc::Math::map3(  0.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f,  500.f, Espfc::Math::map3( 50.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f, -500.f, Espfc::Math::map3(-50.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
}

void test_math_deadband()
{
    TEST_ASSERT_EQUAL_INT32( 0, Espfc::Math::deadband(  0, 10));
    TEST_ASSERT_EQUAL_INT32( 0, Espfc::Math::deadband( 10, 10));
    TEST_ASSERT_EQUAL_INT32( 1, Espfc::Math::deadband( 11, 10));
    TEST_ASSERT_EQUAL_INT32(-1, Espfc::Math::deadband(-11, 10));
    TEST_ASSERT_EQUAL_INT32( 0, Espfc::Math::deadband( -5, 10));
    TEST_ASSERT_EQUAL_INT32(10, Espfc::Math::deadband( 20, 10));
}

void assert_filter_off(Espfc::Filter& filter)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, filter.update(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, filter.update(0.1f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, filter.update(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, filter.update(1.5f));
}

void test_filter_default()
{
    Espfc::Filter filter;
    assert_filter_off(filter);
}

void test_filter_none()
{
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_NONE, 0);
    filter.begin(config, 100);
    assert_filter_off(filter);
}

void test_filter_pt1_off()
{
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_PT1, 0);
    filter.begin(config, 100);
    assert_filter_off(filter);
}

void test_filter_pt1_50_100()
{
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_PT1, 50);
    filter.begin(config, 100);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.076f, filter.update(0.1f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.397f, filter.update(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.234f, filter.update(1.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.436f, filter.update(1.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.484f, filter.update(1.5f));
}

void test_filter_biquad_off()
{
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_BIQUAD, 0);
    filter.begin(config, 100);
    assert_filter_off(filter);
}

void test_filter_biquad_20_100()
{
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_BIQUAD, 20);
    filter.begin(config, 100);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.020f, filter.update(0.1f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.152f, filter.update(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.589f, filter.update(1.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.220f, filter.update(1.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.575f, filter.update(1.5f));
}

void test_filter_notch_df1_150_200_1000()
{
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_NOTCH_DF1, 200, 150);
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
    Espfc::Filter filter;
    const Espfc::FilterConfig config(Espfc::FILTER_NOTCH_DF1, 200, 150);
    filter.begin(config, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, filter.update(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.783f, filter.update(1.0f));
    filter.reconfigureNotchDF1(200, 150);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.678f, filter.update(1.0f));
    filter.reconfigureNotchDF1(200, 150);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.967f, filter.update(1.0f));
    filter.reconfigureNotchDF1(200, 150);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.166f, filter.update(1.0f));
    filter.reconfigureNotchDF1(200, 150);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.099f, filter.update(1.0f));
}

void test_filter_notch_q()
{
    Espfc::Filter filter;

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.667f, filter.getNotchQ(200, 100));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.667f, filter.getNotchQApprox(200, 100));

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.714f, filter.getNotchQ(200, 150));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.714f, filter.getNotchQApprox(200, 150));

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.744f, filter.getNotchQ(200, 190));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.744f, filter.getNotchQApprox(200, 190));

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.737f, filter.getNotchQ(100, 90));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.737f, filter.getNotchQApprox(100, 90));

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.733f, filter.getNotchQ(400, 350));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.733f, filter.getNotchQApprox(400, 350));
}

void test_pid_init()
{
    Espfc::Pid pid;
    pid.rate = 100;
    pid.begin();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, pid.rate);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.01f, pid.dt);
}

void ensure(Espfc::Pid& pid, float rate = 100.0f)
{
    pid.rate = rate;
    pid.pScale = 1.0f;
    pid.iScale = 1.0f;
    pid.dScale = 1.0f;
    pid.fScale = 1.0f;
    pid.oLimit = 1.0f;
    pid.iLimit = 0.2f;
}

void gain(Espfc::Pid& pid, float p, float i, float d, float f)
{
    pid.Kp = p;
    pid.Ki = i;
    pid.Kd = d;
    pid.Kf = f;
}

void test_pid_update_p()
{
    Espfc::Pid pid;
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
    Espfc::Pid pid;
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
    Espfc::Pid pid;
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
    Espfc::Pid pid;
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
    Espfc::Pid pid;
    ensure(pid);
    gain(pid, 0, 0, 0, 0.1f);
    pid.begin();

    float result1 = pid.update(-0.05f, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.05f, pid.error);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.05f, pid.prevSetpoint);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.fTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, result1);

    float result2 = pid.update(0.0f, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.error);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.prevMeasure);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.pTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, pid.fTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, result2);
}

void test_pid_update_sum()
{
    Espfc::Pid pid;
    ensure(pid);
    gain(pid, 1, 100, 0.1f, 0.1f);
    pid.begin();

    float result = pid.update(0.1f, 0.f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.error);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.pTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, pid.fTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.8f, result);
}

void test_pid_update_sum_limit()
{
    Espfc::Pid pid;
    ensure(pid);
    gain(pid, 3, 100, 0.01f, 0.01f);
    pid.begin();

    float result = pid.update(0.5f, 0.f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.error);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, pid.pTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.iTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.dTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, pid.fTerm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, result);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_math_map);
    RUN_TEST(test_math_map3);
    RUN_TEST(test_math_deadband);
    RUN_TEST(test_filter_default);
    RUN_TEST(test_filter_none);
    RUN_TEST(test_filter_pt1_off);
    RUN_TEST(test_filter_pt1_50_100);
    RUN_TEST(test_filter_biquad_off);
    RUN_TEST(test_filter_biquad_20_100);
    RUN_TEST(test_filter_notch_df1_150_200_1000);
    RUN_TEST(test_filter_notch_df1_150_200_1000_reconf);
    RUN_TEST(test_filter_notch_q);
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