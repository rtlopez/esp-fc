#include "BlackboxBridge.h"

static Espfc::Model * _model_ptr = nullptr;

void initBlackboxModel(Espfc::Model * m)
{
  _model_ptr = m;
}

uint16_t getBatteryVoltageLatest(void)
{
  if(!_model_ptr) return 0;
  float v = (*_model_ptr).state.battery.voltageUnfiltered;
  return constrain(lrintf(v * 100.0f), 0, 32000);
}

int32_t getAmperageLatest(void)
{
  if(!_model_ptr) return 0;
  float v = (*_model_ptr).state.battery.currentUnfiltered;
  return constrain(lrintf(v * 100.0f), 0, 32000);
}

bool rxIsReceivingSignal(void)
{
  if(!_model_ptr) return false;
  return !((*_model_ptr).state.inputRxLoss || (*_model_ptr).state.inputRxFailSafe);
}

bool isRssiConfigured(void)
{
  if(!_model_ptr) return false;
  return (*_model_ptr).config.input.rssiChannel > 0;
}

uint16_t getRssi(void)
{
  if(!_model_ptr) return 0;
  return (*_model_ptr).getRssi();
}

failsafePhase_e failsafePhase()
{
  if(!_model_ptr) return ::FAILSAFE_IDLE;
  return (failsafePhase_e)(*_model_ptr).state.failsafe.phase;
}

static uint32_t activeFeaturesLatch = 0;
static uint32_t enabledSensors = 0;

bool featureIsEnabled(uint32_t mask)
{
  return activeFeaturesLatch & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

bool sensors(uint32_t mask)
{
  return enabledSensors & mask;
}

float pidGetPreviousSetpoint(int axis)
{
  return Espfc::Math::toDeg(_model_ptr->state.desiredRate[axis]);
}

float mixerGetThrottle(void)
{
  return (_model_ptr->state.output[Espfc::AXIS_THRUST] + 1.0f) * 0.5f;
}

int16_t getMotorOutputLow()
{
  return _model_ptr->state.digitalOutput ? PWM_TO_DSHOT(1000) : 1000;
}

int16_t getMotorOutputHigh()
{
  return _model_ptr->state.digitalOutput ? PWM_TO_DSHOT(2000) : 2000;
}

bool areMotorsRunning(void)
{
  return _model_ptr->areMotorsRunning();
}

uint16_t getDshotErpm(uint8_t i)
{
  return _model_ptr->state.outputTelemetryErpm[i];
}
