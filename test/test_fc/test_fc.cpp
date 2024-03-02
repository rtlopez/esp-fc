#include <unity.h>
#include <ArduinoFake.h>
#include <EscDriver.h>
#include "Timer.h"
#include "Model.h"
#include "Controller.h"
#include "Actuator.h"
#include "Output/Mixer.h"

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
  model.state.gyroClock = 8000;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.begin();

  TEST_ASSERT_EQUAL_INT32(8000, model.state.gyroClock);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.gyroRate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.gyroTimer.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.loopRate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.loopTimer.rate);
  TEST_ASSERT_EQUAL_INT32(2000, model.state.mixerTimer.rate);
}

void test_model_gyro_init_1k_188dlpf()
{
  Model model;
  model.state.gyroClock = 1000;
  model.config.gyroDlpf = GYRO_DLPF_188;
  model.config.loopSync = 2;
  model.config.mixerSync = 2;
  model.begin();

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
  model.state.gyroClock = 1000;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixerType = FC_MIXER_QUADX;
  model.config.pid[FC_PID_ROLL]  = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.config.pid[FC_PID_PITCH] = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.config.pid[FC_PID_YAW]   = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.begin();

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[FC_PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[FC_PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_PITCH].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 1000.0f, model.state.innerPid[FC_PID_YAW].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1835f, model.state.innerPid[FC_PID_YAW].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.4002f, model.state.innerPid[FC_PID_YAW].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0030f, model.state.innerPid[FC_PID_YAW].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000788f, model.state.innerPid[FC_PID_YAW].Kf);
}

void test_model_outer_pid_init()
{
  Model model;
  model.state.gyroClock = 8000;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.loopSync = 1;
  model.config.mixerSync = 1;
  model.config.mixerType = FC_MIXER_QUADX;
  model.config.pid[FC_PID_LEVEL]  = { .P = 100u, .I = 100u, .D = 100u, .F = 100 };
  model.begin();

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 2000.0f, model.state.outerPid[FC_PID_ROLL].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_ROLL].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_ROLL].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_ROLL].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_ROLL].Kf);

  TEST_ASSERT_FLOAT_WITHIN(   0.1f, 2000.0f, model.state.outerPid[FC_PID_PITCH].rate);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_PITCH].Kp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,   10.0f, model.state.outerPid[FC_PID_PITCH].Ki);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_PITCH].Kd);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,    0.1f, model.state.outerPid[FC_PID_PITCH].Kf);
}

void test_controller_rates()
{
  Model model;
  model.state.gyroClock = 8000;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.loopSync = 8;
  model.config.mixerSync = 1;
  model.config.mixerType = FC_MIXER_QUADX;

  model.config.input.rateType = RATES_TYPE_BETAFLIGHT;
  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rateLimit[AXIS_ROLL] = 1998;

  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rateLimit[AXIS_PITCH] = 1998;
  
  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;
  model.config.input.rateLimit[AXIS_YAW] = 1998;

  model.begin();

  Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.76f, controller.calculateSetpointRate(AXIS_ROLL, 0.25f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   4.81f, controller.calculateSetpointRate(AXIS_ROLL, 0.75f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   6.95f, controller.calculateSetpointRate(AXIS_ROLL, 0.85f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, controller.calculateSetpointRate(AXIS_ROLL, 1.1f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, controller.calculateSetpointRate(AXIS_PITCH,  1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.48f, controller.calculateSetpointRate(AXIS_YAW, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -3.65f, controller.calculateSetpointRate(AXIS_YAW, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -8.64f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

void test_controller_rates_limit()
{
  Model model;
  model.state.gyroClock = 8000;
  model.config.gyroDlpf = GYRO_DLPF_256;
  model.config.loopSync = 8;
  model.config.mixerSync = 1;
  model.config.mixerType = FC_MIXER_QUADX;

  model.config.input.rateType = RATES_TYPE_BETAFLIGHT;
  model.config.input.rate[AXIS_ROLL] = 70;
  model.config.input.expo[AXIS_ROLL] = 0;
  model.config.input.superRate[AXIS_ROLL] = 80;
  model.config.input.rateLimit[AXIS_ROLL] = 500;

  model.config.input.rate[AXIS_PITCH] = 70;
  model.config.input.expo[AXIS_PITCH] = 0;
  model.config.input.superRate[AXIS_PITCH] = 80;
  model.config.input.rateLimit[AXIS_PITCH] = 500;

  model.config.input.rate[AXIS_YAW] = 120;
  model.config.input.expo[AXIS_YAW] = 0;
  model.config.input.superRate[AXIS_YAW] = 50;
  model.config.input.rateLimit[AXIS_YAW] = 400;

  model.begin();

  Controller controller(model);
  controller.begin();

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.73f, controller.calculateSetpointRate(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, controller.calculateSetpointRate(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, controller.calculateSetpointRate(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.73f, controller.calculateSetpointRate(AXIS_PITCH,  1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -8.73f, controller.calculateSetpointRate(AXIS_PITCH, -1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, controller.calculateSetpointRate(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.83f, controller.calculateSetpointRate(AXIS_YAW, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -6.98f, controller.calculateSetpointRate(AXIS_YAW, 1.0f));
}

void test_rates_betaflight()
{
  InputConfig config;
  config.rateType = RATES_TYPE_BETAFLIGHT;

  config.rate[AXIS_ROLL]      = config.rate[AXIS_PITCH]      = config.rate[AXIS_YAW]      =   70;
  config.expo[AXIS_ROLL]      = config.expo[AXIS_PITCH]      = config.expo[AXIS_YAW]      =    0;
  config.superRate[AXIS_ROLL] = config.superRate[AXIS_PITCH] = config.superRate[AXIS_YAW] =   80;
  config.rateLimit[AXIS_ROLL] = config.rateLimit[AXIS_PITCH] = config.rateLimit[AXIS_YAW] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_PITCH,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_PITCH,  0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, rates.getSetpoint(AXIS_PITCH, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_PITCH,  1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, rates.getSetpoint(AXIS_PITCH, -1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_YAW, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_YAW, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_YAW, 1.0f));
}

void test_rates_betaflight_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_BETAFLIGHT;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =   10;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.26f, rates.getSetpoint(AXIS_ROLL, 0.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.57f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.95f, rates.getSetpoint(AXIS_ROLL, 0.3f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.40f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.98f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.73f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   3.78f, rates.getSetpoint(AXIS_ROLL, 0.7f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.37f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.10f, rates.getSetpoint(AXIS_ROLL, 0.9f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.51f, rates.getSetpoint(AXIS_ROLL, 1.0f));
}

void test_rates_raceflight()
{
  InputConfig config;
  config.rateType = RATES_TYPE_RACEFLIGHT;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =    0;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.59f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.46f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.90f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.75f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_raceflight_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_RACEFLIGHT;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =   20;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.56f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.35f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.88f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.58f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.04f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.44f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.88f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.44f, rates.getSetpoint(AXIS_ROLL, -1.0f));

}

void test_rates_kiss()
{
  InputConfig config;
  config.rateType = RATES_TYPE_KISS;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =    0;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.59f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.46f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.08f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.90f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.75f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.57f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -2.08f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.57f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_rates_kiss_expo()
{
  InputConfig config;
  config.rateType = RATES_TYPE_KISS;

  config.rate[AXIS_ROLL]      =   70;
  config.expo[AXIS_ROLL]      =   20;
  config.superRate[AXIS_ROLL] =   80;
  config.rateLimit[AXIS_ROLL] = 1998;

  Rates rates;
  rates.begin(config);

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   0.56f, rates.getSetpoint(AXIS_ROLL, 0.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.35f, rates.getSetpoint(AXIS_ROLL, 0.4f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   1.88f, rates.getSetpoint(AXIS_ROLL, 0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   2.58f, rates.getSetpoint(AXIS_ROLL, 0.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,   5.04f, rates.getSetpoint(AXIS_ROLL, 0.8f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  13.45f, rates.getSetpoint(AXIS_ROLL, 1.0f));

  TEST_ASSERT_FLOAT_WITHIN(0.01f,    0.0f, rates.getSetpoint(AXIS_ROLL,  0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f,  -1.88f, rates.getSetpoint(AXIS_ROLL, -0.5f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -13.45f, rates.getSetpoint(AXIS_ROLL, -1.0f));
}

void test_actuator_arming_gyro_motor_calbration()
{
  Model model;
  //model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_NO_GYRO | ARMING_DISABLED_MOTOR_PROTOCOL, model.state.armingDisabledFlags);
}

void test_actuator_arming_failsafe()
{
  Model model;
  model.state.gyroPresent = true;
  model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  model.state.gyroCalibrationState = CALIBRATION_UPDATE;
  model.state.inputRxFailSafe = true;
  model.state.inputRxLoss = true;

  //model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_CALIBRATING, model.state.armingDisabledFlags);
}

void test_actuator_arming_throttle()
{
  Model model;
  model.config.output.protocol = ESC_PROTOCOL_DSHOT150;
  model.config.input.minCheck = 1050;
  model.state.inputUs[AXIS_THRUST] = 1100;
  model.state.gyroPresent = true;

  //model.begin();

  Actuator actuator(model);
  actuator.begin();

  TEST_ASSERT_EQUAL_UINT32(0, model.state.armingDisabledFlags);

  actuator.updateArmingDisabled();

  TEST_ASSERT_EQUAL_UINT32(ARMING_DISABLED_THROTTLE, model.state.armingDisabledFlags);
}

void test_mixer_throttle_limit_none()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_NONE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_NONE, 100));
}

void test_mixer_throttle_limit_scale()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 110));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.00f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.20f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.20f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_SCALE, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.60f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_SCALE, 80));
}

void test_mixer_throttle_limit_clip()
{
  Model model;
  Output::Mixer mixer(model);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 110));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 110));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.00f, mixer.limitThrust(-1.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.00f, mixer.limitThrust( 0.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.50f, mixer.limitThrust( 0.5f, THROTTLE_LIMIT_TYPE_CLIP, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.60f, mixer.limitThrust( 1.0f, THROTTLE_LIMIT_TYPE_CLIP, 80));
}

void test_mixer_output_limit_motor()
{
  Model model;
  Output::Mixer mixer(model);
  OutputChannelConfig motor = { .servo = false };

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, motor, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, motor, 120));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, motor, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, motor, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.6f, mixer.limitOutput( 1.0f, motor, 80));
}

void test_mixer_output_limit_servo()
{
  Model model;
  Output::Mixer mixer(model);
  OutputChannelConfig servo = { .servo = true  };

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, servo, 100));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 120));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, servo, 120));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, mixer.limitOutput(-1.0f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  1.0f, mixer.limitOutput( 1.0f, servo, 0));

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.8f, mixer.limitOutput(-1.0f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, mixer.limitOutput( 0.0f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.5f, mixer.limitOutput( 0.5f, servo, 80));
  TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.8f, mixer.limitOutput( 1.0f, servo, 80));
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
  RUN_TEST(test_controller_rates_limit);
  RUN_TEST(test_rates_betaflight);
  RUN_TEST(test_rates_betaflight_expo);
  RUN_TEST(test_rates_raceflight);
  RUN_TEST(test_rates_raceflight_expo);
  RUN_TEST(test_rates_kiss);
  RUN_TEST(test_rates_kiss_expo);
  RUN_TEST(test_actuator_arming_gyro_motor_calbration);
  RUN_TEST(test_actuator_arming_failsafe);
  RUN_TEST(test_actuator_arming_throttle);
  RUN_TEST(test_mixer_throttle_limit_none);
  RUN_TEST(test_mixer_throttle_limit_scale);
  RUN_TEST(test_mixer_throttle_limit_clip);
  RUN_TEST(test_mixer_output_limit_motor);
  RUN_TEST(test_mixer_output_limit_servo);
  UNITY_END();

  return 0;
}
