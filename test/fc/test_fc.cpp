#include <unity.h>
#include <ArduinoFake.h>
#include "Timer.h"
#include "Model.h"
#include "Controller.h"

using namespace fakeit;
using namespace Espfc;

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

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

  TEST_ASSERT_TRUE( timer.check(1000));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);

  TEST_ASSERT_FALSE(timer.check(1500));
  TEST_ASSERT_EQUAL_UINT32(1, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_TRUE( timer.check(2000));
  TEST_ASSERT_EQUAL_UINT32(2, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_TRUE( timer.check(3000));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_FALSE(timer.check(3999));
  TEST_ASSERT_EQUAL_UINT32(3, timer.iteration);
  TEST_ASSERT_EQUAL_UINT32(1000, timer.delta);
  
  TEST_ASSERT_TRUE( timer.check(4050));
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

  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_TRUE( timer.check());
  TEST_ASSERT_FALSE(timer.check());
  TEST_ASSERT_TRUE( timer.check());

  Verify(Method(ArduinoFake(), micros)).Exactly(6_Times);
}

void test_model_gyro_init_1k_256dlpf()
{
  Model model;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.gyroSync = 8; // 1khz
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.begin();
  model.update();

  TEST_ASSERT_EQUAL_INT32(   8, model.state.gyroDivider);
  TEST_ASSERT_EQUAL_INT32(8000, model.state.gyroClock);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyroRate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyroTimer.rate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.mixerTimer.rate);
}

void test_model_gyro_init_1k_188dlpf()
{
  Model model;
  model.config.gyroDlpf = GYRO_DLPF_188;
  model.config.gyroSync = 8; // 1khz
  model.config.loopSync = 2;
  model.config.mixerSync = 2;
  model.begin();
  model.update();

  TEST_ASSERT_EQUAL_INT32(   1, model.state.gyroDivider);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyroClock);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyroRate);
  TEST_ASSERT_EQUAL_INT32(1000, model.state.gyroTimer.rate);
  TEST_ASSERT_EQUAL_INT32( 500, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32( 500, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32( 250, model.state.mixerTimer.rate);
}

void test_model_inner_pid_init()
{
  Model model;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.gyroSync = 8; // 1khz
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixerType = MIXER_QUADX;
  model.config.pid[PID_ROLL]  = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.config.pid[PID_PITCH] = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.config.pid[PID_YAW]   = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.begin();
  model.update();

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[PID_PITCH].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[PID_YAW].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[PID_YAW].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[PID_YAW].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[PID_YAW].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[PID_YAW].Kf);
}

void test_model_outer_pid_init()
{
  Model model;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.gyroSync = 8; // 1khz
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixerType = MIXER_QUADX;
  model.config.pid[PID_LEVEL]  = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.begin();
  model.update();

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.outerPid[PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.outerPid[PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[PID_PITCH].Kf);
}

void test_controller_rates()
{
  Model model;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.gyroSync = 8; // 1khz
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixerType = MIXER_QUADX;

  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;

  model.begin();
  model.update();

  Controller controller(model);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.76f, controller.calculateSetpointRate(AXIS_ROLL, 0.25f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.04f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   4.58f, controller.calculateSetpointRate(AXIS_ROLL, 0.75f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   6.49f, controller.calculateSetpointRate(AXIS_ROLL, 0.85f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  11.92f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  11.92f, controller.calculateSetpointRate(AXIS_ROLL, 1.1f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.04f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -11.92f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.04f, controller.calculateSetpointRate(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  11.92f, controller.calculateSetpointRate(AXIS_PITCH,  1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.48f, controller.calculateSetpointRate(AXIS_YAW, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -3.59f, controller.calculateSetpointRate(AXIS_YAW, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -8.29f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_timer_rate_100hz);
  RUN_TEST(test_timer_rate_100hz_div2);
  RUN_TEST(test_timer_interval_10ms);
  RUN_TEST(test_timer_check);
  RUN_TEST(test_timer_check_micros);
  RUN_TEST(test_model_gyro_init_1k_256dlpf);
  RUN_TEST(test_model_gyro_init_1k_188dlpf);
  RUN_TEST(test_model_inner_pid_init);
  RUN_TEST(test_model_outer_pid_init);
  RUN_TEST(test_controller_rates);
  UNITY_END();

  return 0;
}