#include <unity.h>
#include <ArduinoFake.h>
#include <Timer.h>

using namespace fakeit;

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

void test_timer_rate_100hz()
{
  Espfc::Timer timer;
  timer.setRate(100);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
}

void test_timer_interval_10ms()
{
  Espfc::Timer timer;
  timer.setInterval(10000);
  TEST_ASSERT_EQUAL_UINT32(100, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(10000, timer.interval);
}

void test_timer_check()
{
  Espfc::Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE( timer.check(1000));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);

  TEST_ASSERT_FALSE(timer.check(1500));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  
  TEST_ASSERT_TRUE( timer.check(2000));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  
  TEST_ASSERT_TRUE( timer.check(3000));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  
  TEST_ASSERT_FALSE(timer.check(3999));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  
  TEST_ASSERT_TRUE( timer.check(4050));
  TEST_ASSERT_EQUAL_UINT32(4, timer.iteration);
}

void test_timer_check_micros()
{
  When(Method(ArduinoFake(), micros)).Return(1000, 1500, 2000, 3000, 3999, 4050);

  Espfc::Timer timer;
  timer.setInterval(1000);

  TEST_ASSERT_EQUAL_UINT32(1000, timer.rate);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.interval);

  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE( timer.check());

  Verify(Method(ArduinoFake(), micros)).Exactly(6_Times);
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_timer_rate_100hz);
  RUN_TEST(test_timer_interval_10ms);
  RUN_TEST(test_timer_check);
  RUN_TEST(test_timer_check_micros);
  UNITY_END();

  return 0;
}