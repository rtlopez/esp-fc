#include "BlackboxBridge.hpp"

#include "Hal/FastCode.hpp"
#include <algorithm>

static Espfc::Model* _model_ptr = nullptr;

void initBlackboxModel(Espfc::Model* m)
{
  _model_ptr = m;
}

uint16_t FAST_CODE_ATTR getBatteryVoltageLatest(void)
{
  if (!_model_ptr) return 0;
  float v = (*_model_ptr).state.battery.voltageUnfiltered;
  return std::clamp<long>(lrintf(v * 100.0f), 0L, 32000L);
}

int32_t FAST_CODE_ATTR getAmperageLatest(void)
{
  if (!_model_ptr) return 0;
  float v = (*_model_ptr).state.battery.currentUnfiltered;
  return std::clamp<long>(lrintf(v * 100.0f), 0L, 32000L);
}

bool FAST_CODE_ATTR isRxReceivingSignal(void)
{
  if (!_model_ptr) return false;
  return !((*_model_ptr).state.input.rxLoss || (*_model_ptr).state.input.rxFailSafe);
}

bool FAST_CODE_ATTR rxAreFlightChannelsValid(void)
{
  if (!_model_ptr) return false;
  return (*_model_ptr).state.input.channelsValid;
}

bool FAST_CODE_ATTR isRssiConfigured(void)
{
  if (!_model_ptr) return false;
  return (*_model_ptr).config.input.rssiChannel > 0;
}

uint16_t FAST_CODE_ATTR getRssi(void)
{
  if (!_model_ptr) return 0;
  return (*_model_ptr).getRssi();
}

failsafePhase_e FAST_CODE_ATTR failsafePhase()
{
  if (!_model_ptr) return ::FAILSAFE_IDLE;
  return (failsafePhase_e)(*_model_ptr).state.failsafe.phase;
}

static uint32_t enabledSensors = 0;

bool FAST_CODE_ATTR featureIsEnabled(uint32_t mask)
{
  return featureConfigMutable()->enabledFeatures & mask;
}

void sensorsSet(uint32_t mask)
{
  enabledSensors |= mask;
}

bool FAST_CODE_ATTR sensors(uint32_t mask)
{
  return enabledSensors & mask;
}

float FAST_CODE_ATTR pidGetPreviousSetpoint(int axis)
{
  return Espfc::Utils::toDeg(_model_ptr->state.setpoint.rate[axis]);
}

float FAST_CODE_ATTR mixerGetThrottle(void)
{
  return (_model_ptr->state.output.ch[Espfc::AXIS_THRUST] + 1.0f) * 0.5f;
}

int16_t FAST_CODE_ATTR getMotorOutputLow()
{
  return _model_ptr->state.mixer.digitalOutput ? PWM_TO_DSHOT(1000) : 1000;
}

int16_t FAST_CODE_ATTR getMotorOutputHigh()
{
  return _model_ptr->state.mixer.digitalOutput ? PWM_TO_DSHOT(2000) : 2000;
}

bool FAST_CODE_ATTR areMotorsRunning(void)
{
  return _model_ptr->areMotorsRunning() || _model_ptr->state.mode.isLongClickActive();
}

uint16_t FAST_CODE_ATTR getDshotErpm(uint8_t i)
{
  return _model_ptr->state.output.telemetry.erpm[i];
}
