#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_

#include "Model.h"
#include "Hardware.h"

extern "C" {
#include "blackbox/blackbox.h"

static Espfc::SerialDevice * blackboxSerial = NULL;
static Espfc::Model * _model_ptr = NULL;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
  UNUSED(instance);
  if(blackboxSerial) blackboxSerial->write(ch);
}

void serialWriteInit(Espfc::SerialDevice * serial)
{
  blackboxSerial = serial;
}

uint32_t serialTxBytesFree(const serialPort_t *instance)
{
  UNUSED(instance);
  if(!blackboxSerial) return 0;
  return blackboxSerial->availableForWrite();
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
  UNUSED(instance);
  if(!blackboxSerial) return false;
  return blackboxSerial->availableForWrite() > 127;
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
  return ((*_model_ptr).state.inputLinkValid);
}

static uint32_t activeFeaturesLatch = 0;
static uint32_t enabledSensors = 0;

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

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

      _serial = Hardware::getSerialPort(_model.config.serial, SERIAL_FUNCTION_BLACKBOX);
      if(!_serial) return 0;

      serialWriteInit(_serial);

      systemConfigMutable()->activeRateProfile = 0;
      systemConfigMutable()->debug_mode = debugMode = _model.config.debugMode;

      controlRateConfig_t *rp = controlRateProfilesMutable(systemConfig()->activeRateProfile);
      rp->rcRate8 = _model.config.input.rate[ROLL];
      rp->rcExpo8 = _model.config.input.expo[ROLL];
      rp->rcYawRate8 = _model.config.input.rate[YAW];
      rp->rcYawExpo8 = _model.config.input.expo[YAW];
      rp->thrMid8 = 50;
      rp->thrExpo8 = 0;
      rp->dynThrPID = _model.config.tpaScale;
      rp->tpa_breakpoint = _model.config.tpaBreakpoint;
      rp->rates[ROLL] = _model.config.input.superRate[ROLL];
      rp->rates[PITCH] = _model.config.input.superRate[PITCH];
      rp->rates[YAW] = _model.config.input.superRate[YAW];

      pidProfile_s * cp = currentPidProfile = &_pidProfile;
      for(size_t i = 0; i < PID_ITEM_COUNT; i++)
      {
        cp->pid[i].P = _model.config.pid[i].P;
        cp->pid[i].I = _model.config.pid[i].I;
        cp->pid[i].D = _model.config.pid[i].D;
      }
      cp->pidAtMinThrottle = 1;
      cp->dterm_filter_type = _model.config.dtermFilter.type;
      cp->dterm_lpf_hz = _model.config.dtermFilter.freq;
      cp->dterm_notch_hz = _model.config.dtermNotchFilter.freq;
      cp->dterm_notch_cutoff = _model.config.dtermNotchFilter.cutoff;
      cp->yaw_lpf_hz = _model.config.yawFilter.freq;
      cp->itermWindupPointPercent = _model.config.itermWindupPointPercent;
      cp->dtermSetpointWeight = _model.config.dtermSetpointWeight;

      rcControlsConfigMutable()->deadband = _model.config.input.deadband;
      rcControlsConfigMutable()->yaw_deadband = _model.config.input.deadband;

      gyroConfigMutable()->gyro_lpf = _model.config.gyroDlpf;
      gyroConfigMutable()->gyro_soft_lpf_type = _model.config.gyroFilter.type;
      gyroConfigMutable()->gyro_soft_lpf_hz = _model.config.gyroFilter.freq;
      gyroConfigMutable()->gyro_soft_notch_cutoff_1 = _model.config.gyroNotch1Filter.cutoff;
      gyroConfigMutable()->gyro_soft_notch_hz_1 = _model.config.gyroNotch1Filter.freq;
      gyroConfigMutable()->gyro_soft_notch_cutoff_2 = _model.config.gyroNotch2Filter.cutoff;
      gyroConfigMutable()->gyro_soft_notch_hz_2 = _model.config.gyroNotch2Filter.freq;

      accelerometerConfigMutable()->acc_lpf_hz = _model.config.accelFilter.freq;
      accelerometerConfigMutable()->acc_hardware = _model.config.accelDev;
      blackboxConfigMutable()->record_acc = _model.config.accelDev != ACCEL_NONE && _model.config.accelMode != ACCEL_OFF;
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
      motorConfigMutable()->minthrottle = motorOutputLow = _model.state.minThrottle;
      motorConfigMutable()->maxthrottle = motorOutputHigh = _model.state.maxThrottle;
      if(_model.state.digitalOutput)
      {
        motorOutputLow = (_model.state.minThrottle - 1000) * 2 + 47;
        motorOutputHigh = (_model.state.maxThrottle - 1000) * 2 + 47;
      }

      blackboxConfigMutable()->p_denom = _model.config.blackboxPdenom;
      blackboxConfigMutable()->device = _model.config.blackboxDev;

      gyroConfigMutable()->gyro_sync_denom = _model.config.gyroSync;
      pidConfigMutable()->pid_process_denom = _model.config.loopSync;

      targetPidLooptime = _model.state.loopTimer.interval;
      gyro.targetLooptime = _model.state.gyroTimer.interval;

      featureConfigMutable()->enabledFeatures = _model.config.featureMask;

      batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE; //VOLTAGE_METER_ADC;
      batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;

      rxConfigMutable()->rcInterpolation = _model.config.input.interpolationMode;
      rxConfigMutable()->rcInterpolationInterval = _model.config.input.interpolationInterval;
      rxConfigMutable()->rssi_channel = 0;

      blackboxInit();

      return 1;
    }

    int update()
    {
      if(!_serial) return 0;
      _model.state.stats.start(COUNTER_BLACKBOX);
      updateArmed();
      updateMode();
      updateData();
      blackboxUpdate(_model.state.loopTimer.last);
      _model.state.stats.end(COUNTER_BLACKBOX);
      return 1;
    }

  private:
    void updateData()
    {
      for(size_t i = 0; i < 3; i++)
      {
        gyro.gyroADCf[i] = degrees(_model.state.gyro[i]);
        acc.accSmooth[i] = _model.state.accel[i] * 2048.f;
        axisPID_P[i] = _model.state.innerPid[i].pTerm * 1000.f;
        axisPID_I[i] = _model.state.innerPid[i].iTerm * 1000.f;
        axisPID_D[i] = _model.state.innerPid[i].dTerm * 1000.f;
        rcCommand[i] = _model.state.input[i] * (i == AXIS_YAW ? -500.f : 500.f);
      }
      rcCommand[AXIS_THRUST] = _model.state.input[AXIS_THRUST] * 500.f + 1500.f;
      for(size_t i = 0; i < 4; i++)
      {
        motor[i] = Math::bound((int)_model.state.outputUs[i], 1000, 2000);
        debug[i] = _model.state.debug[i];
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
      if(_model.isActive(MODE_ARMED)) bitArraySet(&rcModeActivationMask, BOXARM);
      else bitArrayClr(&rcModeActivationMask, BOXARM);

      if(_model.isActive(MODE_ANGLE)) bitArraySet(&rcModeActivationMask, BOXANGLE);
      else bitArrayClr(&rcModeActivationMask, BOXANGLE);

      if(_model.isActive(MODE_AIRMODE)) bitArraySet(&rcModeActivationMask, BOXAIRMODE);
      else bitArrayClr(&rcModeActivationMask, BOXAIRMODE);
    }

    Model& _model;
    pidProfile_s _pidProfile;
    SerialDevice * _serial;
};

}
#endif
