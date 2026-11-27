#include "Utils/Sma.ipp"
#include "Utils/Stats.hpp"
#include "Utils/Timer.hpp"
#include <ArduinoFake.h>
#include <unity.h>

using namespace fakeit;
using namespace Espfc;
using namespace Espfc::Utils;

void test_util_sma_default_window()
{
  static Sma<float, 4> sma;
  TEST_ASSERT_EQUAL_FLOAT(1.f, sma.update(4.f));
  TEST_ASSERT_EQUAL_FLOAT(3.f, sma.update(8.f));
  TEST_ASSERT_EQUAL_FLOAT(6.f, sma.update(12.f));
  TEST_ASSERT_EQUAL_FLOAT(10.f, sma.update(16.f));
}

void test_util_sma_window_slides()
{
  static Sma<float, 4> sma;
  sma.begin(2);
  TEST_ASSERT_EQUAL_FLOAT(5.f, sma.update(10.f));
  TEST_ASSERT_EQUAL_FLOAT(15.f, sma.update(20.f));
  TEST_ASSERT_EQUAL_FLOAT(25.f, sma.update(30.f));
  TEST_ASSERT_EQUAL_FLOAT(35.f, sma.update(40.f));
}

void test_util_sma_count_clamped_to_max()
{
  static Sma<float, 2> sma;
  sma.begin(10);
  TEST_ASSERT_EQUAL_FLOAT(5.f, sma.update(10.f));
  TEST_ASSERT_EQUAL_FLOAT(15.f, sma.update(20.f));
  TEST_ASSERT_EQUAL_FLOAT(25.f, sma.update(30.f));
}

void test_util_sma_passthrough()
{
  static Sma<float, 4> sma;
  sma.begin(1);
  TEST_ASSERT_EQUAL_FLOAT(7.f, sma.update(7.f));
  TEST_ASSERT_EQUAL_FLOAT(-3.f, sma.update(-3.f));
}

void test_timer_rate_100hz()
{
  Timer timer;
  timer.setRate(100);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 100.f, timer.intervalf);
}

void test_timer_rate_100hz_div2()
{
  Timer timer;
  timer.setRate(100, 2);
  TEST_ASSERT_EQUAL_UINT32(50, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(20000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(20000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 50.f, timer.intervalf);
}

void test_timer_interval_10ms()
{
  Timer timer;
  timer.setInterval(10000);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 1.f / 100.f, timer.intervalf);
}

void test_timer_check()
{
  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE(timer.check(1000));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_FALSE(timer.check(1500));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_TRUE(timer.check(2000));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_TRUE(timer.check(3000));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_FALSE(timer.check(3999));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_TRUE(timer.check(4050));
  TEST_ASSERT_EQUAL_UINT32(4, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1050, timer.delta);
}

void test_timer_check_micros()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 1500, 2000, 3000, 3999, 4050);

  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE(timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE(timer.check());
  TEST_ASSERT_TRUE(timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE(timer.check());

  Verify(Method(ArduinoFake(), micros)).Exactly(6_Times);
}

void test_timer_update()
{
  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_INT(1, timer.update(500));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(500, timer.last);
  TEST_ASSERT_EQUAL_UINT32(1500, timer.next);
  TEST_ASSERT_EQUAL_UINT32(500, timer.delta);

  TEST_ASSERT_EQUAL_INT(1, timer.update(700));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(700, timer.last);
  TEST_ASSERT_EQUAL_UINT32(1700, timer.next);
  TEST_ASSERT_EQUAL_UINT32(200, timer.delta);
}

void test_timer_update_micros()
{
  When(Method(ArduinoFake(), micros)).Return(2000, 3500);

  Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_INT(1, timer.update());
  TEST_ASSERT_EQUAL_UINT32(2000, timer.last);
  TEST_ASSERT_EQUAL_UINT32(3000, timer.next);
  TEST_ASSERT_EQUAL_INT(1, timer.update());
  TEST_ASSERT_EQUAL_UINT32(3500, timer.last);
  TEST_ASSERT_EQUAL_UINT32(1500, timer.delta);
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
}

void test_timer_check_not_configured()
{
  Timer timer;
  TEST_ASSERT_FALSE(timer.check(1000));
  TEST_ASSERT_EQUAL_UINT32(0, timer.iteration);
}

void test_timer_sync_to()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 2000, 3000);

  Timer src;
  src.setRate(1000);
  Timer timer;
  timer.setRate(1000, 4);
  TEST_ASSERT_EQUAL_UINT32(4, timer.denom);

  src.iteration = 0;
  TEST_ASSERT_TRUE(timer.syncTo(src));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.last);

  src.iteration = 1;
  TEST_ASSERT_FALSE(timer.syncTo(src));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);

  src.iteration = 4;
  TEST_ASSERT_TRUE(timer.syncTo(src));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(2000, timer.last);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
}

void test_timer_sync_to_slot()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 2000, 3000);

  Timer src;
  src.setRate(1000);
  Timer timer;
  timer.setRate(1000, 4);

  src.iteration = 2;
  TEST_ASSERT_FALSE(timer.syncTo(src, 0));
  TEST_ASSERT_TRUE(timer.syncTo(src, 2));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);

  src.iteration = 3;
  TEST_ASSERT_TRUE(timer.syncTo(src, 3));
  TEST_ASSERT_TRUE(timer.syncTo(src, 100));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
}

void test_timer_sync_to_no_denom()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 1500, 2000);

  Timer src;
  Timer timer;
  timer.setInterval(1000);
  timer.denom = 0;

  TEST_ASSERT_TRUE(timer.syncTo(src));
  TEST_ASSERT_FALSE(timer.syncTo(src));
  TEST_ASSERT_TRUE(timer.syncTo(src));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
}

void test_stats_initial_state()
{
  Stats stats;
  TEST_ASSERT_EQUAL_UINT32(0, stats.loopTime());
  for (int i = 0; i < COUNTER_COUNT; i++)
  {
    TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getLoad((StatCounter)i));
    TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getTime((StatCounter)i));
    TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getFreq((StatCounter)i));
  }
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getTotalLoad());
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getTotalTime());
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getCpuTime());
}

void test_stats_get_name()
{
  Stats stats;
  TEST_ASSERT_EQUAL_STRING("gyro_r", stats.getName(COUNTER_GYRO_READ));
  TEST_ASSERT_EQUAL_STRING("gyro_f", stats.getName(COUNTER_GYRO_FILTER));
  TEST_ASSERT_EQUAL_STRING("gyro_a", stats.getName(COUNTER_GYRO_FFT));
  TEST_ASSERT_EQUAL_STRING(" rpm_u", stats.getName(COUNTER_RPM_UPDATE));
  TEST_ASSERT_EQUAL_STRING(" acc_r", stats.getName(COUNTER_ACCEL_READ));
  TEST_ASSERT_EQUAL_STRING(" acc_f", stats.getName(COUNTER_ACCEL_FILTER));
  TEST_ASSERT_EQUAL_STRING(" mag_r", stats.getName(COUNTER_MAG_READ));
  TEST_ASSERT_EQUAL_STRING(" mag_f", stats.getName(COUNTER_MAG_FILTER));
  TEST_ASSERT_EQUAL_STRING("baro_p", stats.getName(COUNTER_BARO));
  TEST_ASSERT_EQUAL_STRING(" gps_r", stats.getName(COUNTER_GPS_READ));
  TEST_ASSERT_EQUAL_STRING(" imu_p", stats.getName(COUNTER_IMU_FUSION));
  TEST_ASSERT_EQUAL_STRING(" imu_c", stats.getName(COUNTER_IMU_FUSION2));
  TEST_ASSERT_EQUAL_STRING(" pid_o", stats.getName(COUNTER_OUTER_PID));
  TEST_ASSERT_EQUAL_STRING(" pid_i", stats.getName(COUNTER_INNER_PID));
  TEST_ASSERT_EQUAL_STRING(" mix_p", stats.getName(COUNTER_MIXER));
  TEST_ASSERT_EQUAL_STRING(" mix_w", stats.getName(COUNTER_MIXER_WRITE));
  TEST_ASSERT_EQUAL_STRING(" mix_r", stats.getName(COUNTER_MIXER_READ));
  TEST_ASSERT_EQUAL_STRING(" bblog", stats.getName(COUNTER_BLACKBOX));
  TEST_ASSERT_EQUAL_STRING("  rx_r", stats.getName(COUNTER_INPUT_READ));
  TEST_ASSERT_EQUAL_STRING("  rx_f", stats.getName(COUNTER_INPUT_FILTER));
  TEST_ASSERT_EQUAL_STRING("  rx_s", stats.getName(COUNTER_FAILSAFE));
  TEST_ASSERT_EQUAL_STRING("  rx_a", stats.getName(COUNTER_ACTUATOR));
  TEST_ASSERT_EQUAL_STRING("serial", stats.getName(COUNTER_SERIAL));
  TEST_ASSERT_EQUAL_STRING("  wifi", stats.getName(COUNTER_WIFI));
  TEST_ASSERT_EQUAL_STRING("   bat", stats.getName(COUNTER_BATTERY));
  TEST_ASSERT_EQUAL_STRING("   tlm", stats.getName(COUNTER_TELEMETRY));
  TEST_ASSERT_EQUAL_STRING(" cpu_0", stats.getName(COUNTER_CPU_0));
  TEST_ASSERT_EQUAL_STRING(" cpu_1", stats.getName(COUNTER_CPU_1));
  TEST_ASSERT_EQUAL_STRING("unknwn", stats.getName(COUNTER_COUNT));
}

void test_stats_loop_tick()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 2000);

  Stats stats;
  stats.loopTick();
  TEST_ASSERT_EQUAL_UINT32(125, stats.loopTime());
  stats.loopTick();
  TEST_ASSERT_EQUAL_UINT32(234, stats.loopTime());
}

void test_stats_update()
{
  When(Method(ArduinoFake(), micros)).Return(100, 300, 400, 600, 1000);

  Stats stats;
  stats.timer.setInterval(100000);

  stats.start(COUNTER_GYRO_READ);
  stats.end(COUNTER_GYRO_READ);
  stats.start(COUNTER_GYRO_READ);
  stats.end(COUNTER_GYRO_READ);
  stats.update();

  TEST_ASSERT_EQUAL_UINT32(1000, stats.timer.delta);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 40.1f, stats.getLoad(COUNTER_GYRO_READ));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 401.f, stats.getTime(COUNTER_GYRO_READ));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 200.5f, stats.getReal(COUNTER_GYRO_READ));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2000.f, stats.getFreq(COUNTER_GYRO_READ));
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getLoad(COUNTER_MIXER));
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getReal(COUNTER_MIXER));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 40.1f, stats.getTotalLoad());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 401.f, stats.getTotalTime());
}

void test_stats_update_not_due()
{
  When(Method(ArduinoFake(), micros)).Return(100, 300, 1000, 1100, 1200, 1300, 101000);

  Stats stats;
  stats.timer.setInterval(100000);

  stats.start(COUNTER_MIXER);
  stats.end(COUNTER_MIXER);
  stats.update();
  const float first = stats.getLoad(COUNTER_MIXER);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 20.f, first);

  stats.start(COUNTER_MIXER);
  stats.end(COUNTER_MIXER);
  stats.update();
  TEST_ASSERT_EQUAL_FLOAT(first, stats.getLoad(COUNTER_MIXER));
  TEST_ASSERT_EQUAL_UINT32(1, stats.timer.iteration);
}

void test_stats_measure_and_cpu()
{
  When(Method(ArduinoFake(), micros)).Return(0, 500, 0, 250, 1000);

  Stats stats;
  stats.timer.setInterval(100000);
  {
    Stats::Measure m0(stats, COUNTER_CPU_0);
  }
  {
    Stats::Measure m1(stats, COUNTER_CPU_1);
  }
  stats.update();

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 50.f, stats.getLoad(COUNTER_CPU_0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 25.f, stats.getLoad(COUNTER_CPU_1));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 750.f, stats.getCpuTime());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 41.6667f, stats.getCpuLoad());
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getTotalLoad());
  TEST_ASSERT_EQUAL_FLOAT(0.f, stats.getTotalTime());
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_util_sma_default_window);
  RUN_TEST(test_util_sma_window_slides);
  RUN_TEST(test_util_sma_count_clamped_to_max);
  RUN_TEST(test_util_sma_passthrough);
  RUN_TEST(test_timer_rate_100hz);
  RUN_TEST(test_timer_rate_100hz_div2);
  RUN_TEST(test_timer_interval_10ms);
  RUN_TEST(test_timer_check);
  RUN_TEST(test_timer_check_micros);
  RUN_TEST(test_timer_update);
  RUN_TEST(test_timer_update_micros);
  RUN_TEST(test_timer_check_not_configured);
  RUN_TEST(test_timer_sync_to);
  RUN_TEST(test_timer_sync_to_slot);
  RUN_TEST(test_timer_sync_to_no_denom);
  RUN_TEST(test_stats_initial_state);
  RUN_TEST(test_stats_get_name);
  RUN_TEST(test_stats_loop_tick);
  RUN_TEST(test_stats_update);
  RUN_TEST(test_stats_update_not_due);
  RUN_TEST(test_stats_measure_and_cpu);
  return UNITY_END();
}
