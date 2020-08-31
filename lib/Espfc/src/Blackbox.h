#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_

#include "Model.h"
#include "Hardware.h"
#include "EscDriver.h"
#include "Math/Utils.h"

extern "C" {
#include "blackbox/blackbox.h"
}

static Espfc::Device::SerialDevice * blackboxSerial = nullptr;
static Espfc::Model * _model_ptr = nullptr;

void serialWrite(serialPort_t * instance, uint8_t ch)
{
  UNUSED(instance);
  if(blackboxSerial) blackboxSerial->write(ch);
}

void serialWriteInit(Espfc::Device::SerialDevice * serial)
{
  blackboxSerial = serial;
}

uint32_t serialTxBytesFree(const serialPort_t * instance)
{
  UNUSED(instance);
  if(!blackboxSerial) return 0;
  return blackboxSerial->availableForWrite();
}

bool isSerialTransmitBufferEmpty(const serialPort_t * instance)
{
  UNUSED(instance);
  if(!blackboxSerial) return false;
  return blackboxSerial->isTxFifoEmpty();
}

void initBlackboxModel(Espfc::Model * m)
{
  _model_ptr = m;
}

uint16_t getBatteryVoltageLatest(void)
{
  if(!_model_ptr) return 0;
  return ((*_model_ptr).state.battery.voltage);
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

namespace Espfc {

class Blackbox
{
  public:
    Blackbox(Model& model): _model(model) {}

    int begin()
    {
      initBlackboxModel(&_model);

      if(!_model.blackboxEnabled()) return 0;

      Device::SerialDevice * serial = _model.getSerialStream(SERIAL_FUNCTION_BLACKBOX);
      if(!serial) return 0;

      serialWriteInit(serial);

      systemConfigMutable()->activeRateProfile = 0;
      systemConfigMutable()->debug_mode = debugMode = _model.config.debugMode;

      controlRateConfig_t *rp = controlRateProfilesMutable(systemConfig()->activeRateProfile);
      for(int i = 0; i <= AXIS_YAW; i++)
      {
        rp->rcRates[i] = _model.config.input.rate[i];
        rp->rcExpo[i] = _model.config.input.expo[i];
        rp->rates[i] = _model.config.input.superRate[i];
        rp->rate_limit[i] = _model.config.input.rateLimit[i];
      }
      rp->thrMid8 = 50;
      rp->thrExpo8 = 0;
      rp->dynThrPID = _model.config.tpaScale;
      rp->tpa_breakpoint = _model.config.tpaBreakpoint;
      rp->rates_type = _model.config.input.rateType;

      pidProfile_s * cp = currentPidProfile = &_pidProfile;
      for(size_t i = 0; i < PID_ITEM_COUNT; i++)
      {
        cp->pid[i].P = _model.config.pid[i].P;
        cp->pid[i].I = _model.config.pid[i].I;
        cp->pid[i].D = _model.config.pid[i].D;
        cp->pid[i].F = _model.config.pid[i].F;
      }
      cp->pidAtMinThrottle = 1;
      cp->dterm_filter_type = _model.config.dtermFilter.type;
      cp->dterm_lowpass_hz = _model.config.dtermFilter.freq;
      cp->dterm_lowpass2_hz = _model.config.dtermFilter2.freq;
      cp->dterm_filter2_type = _model.config.dtermFilter2.type;
      cp->dterm_notch_hz = _model.config.dtermNotchFilter.freq;
      cp->dterm_notch_cutoff = _model.config.dtermNotchFilter.cutoff;
      cp->yaw_lowpass_hz = _model.config.yawFilter.freq;
      cp->itermWindupPointPercent = _model.config.itermWindupPointPercent;
      cp->antiGravityMode = 0;
      cp->pidSumLimit = 500;
      cp->pidSumLimitYaw = 500;
      cp->ff_boost = 0;
      cp->dyn_lpf_dterm_min_hz = _model.config.dtermDynLpfFilter.cutoff;
      cp->dyn_lpf_dterm_max_hz = _model.config.dtermDynLpfFilter.freq;
      cp->feedForwardTransition = 0;

      rcControlsConfigMutable()->deadband = _model.config.input.deadband;
      rcControlsConfigMutable()->yaw_deadband = _model.config.input.deadband;

      gyroConfigMutable()->gyro_hardware_lpf = _model.config.gyroDlpf;
      gyroConfigMutable()->gyro_lowpass_type = _model.config.gyroFilter.type;
      gyroConfigMutable()->gyro_lowpass_hz = _model.config.gyroFilter.freq;
      gyroConfigMutable()->gyro_lowpass2_type = _model.config.gyroFilter2.type;
      gyroConfigMutable()->gyro_lowpass2_hz = _model.config.gyroFilter2.freq;
      gyroConfigMutable()->gyro_soft_notch_cutoff_1 = _model.config.gyroNotch1Filter.cutoff;
      gyroConfigMutable()->gyro_soft_notch_hz_1 = _model.config.gyroNotch1Filter.freq;
      gyroConfigMutable()->gyro_soft_notch_cutoff_2 = _model.config.gyroNotch2Filter.cutoff;
      gyroConfigMutable()->gyro_soft_notch_hz_2 = _model.config.gyroNotch2Filter.freq;
      gyroConfigMutable()->dyn_lpf_gyro_min_hz = _model.config.gyroDynLpfFilter.cutoff;
      gyroConfigMutable()->dyn_lpf_gyro_max_hz = _model.config.gyroDynLpfFilter.freq;

      accelerometerConfigMutable()->acc_lpf_hz = _model.config.accelFilter.freq;
      accelerometerConfigMutable()->acc_hardware = _model.config.accelDev;
      blackboxConfigMutable()->record_acc = _model.accelActive();
      if(blackboxConfig()->record_acc)
      {
          enabledSensors |= SENSOR_ACC;
      }
      barometerConfigMutable()->baro_hardware = _model.config.baroDev;
      compassConfigMutable()->mag_hardware = _model.config.magDev;

      motorConfigMutable()->dev.useUnsyncedPwm = _model.config.output.async;
      motorConfigMutable()->dev.motorPwmProtocol = _model.config.output.protocol;
      motorConfigMutable()->dev.motorPwmRate = _model.config.output.rate;
      motorConfigMutable()->mincommand = _model.config.output.minCommand;
      motorConfigMutable()->digitalIdleOffsetValue = _model.config.output.dshotIdle;
      motorConfigMutable()->minthrottle = _model.state.minThrottle;
      motorConfigMutable()->maxthrottle = _model.state.maxThrottle;
      motorOutputLow  = _model.state.digitalOutput ? PWM_TO_DSHOT(1000) : 1000;
      motorOutputHigh = _model.state.digitalOutput ? PWM_TO_DSHOT(2000) : 2000;

      blackboxConfigMutable()->p_ratio = _model.config.blackboxPdenom;
      blackboxConfigMutable()->device = _model.config.blackboxDev;

      gyroConfigMutable()->gyro_sync_denom = _model.config.gyroSync;
      pidConfigMutable()->pid_process_denom = _model.config.loopSync;

      targetPidLooptime = _model.state.loopTimer.interval;
      gyro.sampleLooptime = _model.state.gyroTimer.interval;
      activePidLoopDenom = _model.config.loopSync;

      featureConfigMutable()->enabledFeatures = _model.config.featureMask;

      batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE; //VOLTAGE_METER_ADC;
      batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;

      rxConfigMutable()->rcInterpolation = _model.config.input.interpolationMode;
      rxConfigMutable()->rcInterpolationInterval = _model.config.input.interpolationInterval;
      rxConfigMutable()->rssi_channel = _model.config.input.rssiChannel;
      rxConfigMutable()->airModeActivateThreshold = 40;
      blackboxInit();

      return 1;
    }

    int update()
    {
      if(!_model.blackboxEnabled()) return 0;
      if(!blackboxSerial) return 0;
      Stats::Measure measure(_model.state.stats, COUNTER_BLACKBOX);
      updateArmed();
      updateMode();
      updateData();
      blackboxUpdate(_model.state.loopTimer.last);
      return 1;
    }

  private:
    void updateData()
    {
      for(size_t i = 0; i < 3; i++)
      {
        gyro.gyroADCf[i] = degrees(_model.state.gyro[i]);
        pidData[i].P = _model.state.innerPid[i].pTerm * 1000.f;
        pidData[i].I = _model.state.innerPid[i].iTerm * 1000.f;
        pidData[i].D = _model.state.innerPid[i].dTerm * 1000.f;
        pidData[i].F = _model.state.innerPid[i].fTerm * 1000.f;
        rcCommand[i] = _model.state.input[i] * (i == AXIS_YAW ? -500.f : 500.f);
        if(_model.accelActive()) {
          acc.accADC[i] = _model.state.accel[i] * ACCEL_G_INV * acc.dev.acc_1G;
        }
        if(_model.magActive()) {
          mag.magADC[i] = _model.state.mag[i];
        }
        if(_model.baroActive()) {
          baro.BaroAlt = lrintf(_model.state.baroAltitude * 100.f); // cm
        }
      }
      rcCommand[AXIS_THRUST] = _model.state.input[AXIS_THRUST] * 500.f + 1500.f;
      for(size_t i = 0; i < 4; i++)
      {
        motor[i] = Math::clamp(_model.state.outputUs[i], (int16_t)1000, (int16_t)2000);
        if(_model.state.digitalOutput) {
          motor[i] = PWM_TO_DSHOT(motor[i]);
        }
        if(_model.config.debugMode != DEBUG_NONE && _model.config.debugMode != DEBUG_BLACKBOX_OUTPUT) {
          debug[i] = _model.state.debug[i];
        }
      }
    }

    void updateArmed()
    {
      static uint32_t beep = 0;
      if(beep != 0 && beep < _model.state.loopTimer.last)
      {
        setArmingBeepTimeMicros(_model.state.loopTimer.last);
        beep = 0;
      }

      bool armed =_model.isActive(MODE_ARMED);
      if(armed == ARMING_FLAG(ARMED)) return;
      if(armed)
      {
        ENABLE_ARMING_FLAG(ARMED);
        beep = _model.state.loopTimer.last + 200000; // delay arming beep event ~50ms (200ms)
      }
      else
      {
        DISABLE_ARMING_FLAG(ARMED);
        setArmingBeepTimeMicros(micros());
        blackboxFinish();
      }
    }

    void updateMode()
    {
      if(_model.isSwitchActive(MODE_ARMED)) bitArraySet(&rcModeActivationMask, BOXARM);
      else bitArrayClr(&rcModeActivationMask, BOXARM);

      if(_model.isSwitchActive(MODE_ANGLE)) bitArraySet(&rcModeActivationMask, BOXANGLE);
      else bitArrayClr(&rcModeActivationMask, BOXANGLE);

      if(_model.isSwitchActive(MODE_AIRMODE)) bitArraySet(&rcModeActivationMask, BOXAIRMODE);
      else bitArrayClr(&rcModeActivationMask, BOXAIRMODE);

      if(_model.isSwitchActive(MODE_FAILSAFE)) bitArraySet(&rcModeActivationMask, BOXFAILSAFE);
      else bitArrayClr(&rcModeActivationMask, BOXFAILSAFE);
    }

    Model& _model;
    pidProfile_s _pidProfile;
};

}
#endif
