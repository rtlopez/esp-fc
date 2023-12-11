#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_

#include "Model.h"
#include "Hardware.h"
#include "EscDriver.h"
#include "Math/Utils.h"

extern "C" {
#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"
}

class BlackboxBuffer
{
  public:
    BlackboxBuffer(): _stream(nullptr), _idx(0) {}

    void begin(Espfc::Device::SerialDevice * s)
    {
      _stream = s;
    }

    void write(uint8_t c)
    {
      _data[_idx++] = c;
      if(_idx >= SIZE) flush();
    }

    void flush()
    {
      if(_stream) _stream->write(_data, _idx);
      _idx = 0;
    }

    size_t availableForWrite() const
    {
      return SIZE - _idx;
    }

    size_t isTxFifoEmpty() const
    {
      return _idx == 0;
    }

    static const size_t SIZE = 128;

    Espfc::Device::SerialDevice * _stream;
    size_t _idx;
    uint8_t _data[SIZE];
};

static BlackboxBuffer * blackboxSerial = nullptr;
static Espfc::Model * _model_ptr = nullptr;

void serialWrite(serialPort_t * instance, uint8_t ch)
{
  UNUSED(instance);
  if(blackboxSerial) blackboxSerial->write(ch);
}

void serialWriteInit(BlackboxBuffer * serial)
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

      _buffer.begin(serial);
      serialWriteInit(&_buffer);

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
        if(i <= AXIS_YAW) {
          cp->d_min[i] = _model.config.pid[i].D;
        }
      }
      cp->pidAtMinThrottle = 1;
      cp->dterm_lpf1_type = _model.config.dtermFilter.type;
      cp->dterm_lpf1_static_hz = _model.config.dtermFilter.freq;
      cp->dterm_lpf1_dyn_min_hz = _model.config.dtermDynLpfFilter.cutoff;
      cp->dterm_lpf1_dyn_max_hz = _model.config.dtermDynLpfFilter.freq;
      cp->dterm_lpf2_type = _model.config.dtermFilter2.type;
      cp->dterm_lpf2_static_hz = _model.config.dtermFilter2.freq;
      cp->dterm_notch_hz = _model.config.dtermNotchFilter.freq;
      cp->dterm_notch_cutoff = _model.config.dtermNotchFilter.cutoff;
      cp->yaw_lowpass_hz = _model.config.yawFilter.freq;
      cp->itermWindupPointPercent = _model.config.itermWindupPointPercent;
      cp->antiGravityMode = 0;
      cp->pidSumLimit = 500;
      cp->pidSumLimitYaw = 500;
      cp->ff_boost = 0;
      cp->feedForwardTransition = 0;
      cp->tpa_mode = 0; // PD
      cp->tpa_rate = _model.config.tpaScale;
      cp->tpa_breakpoint = _model.config.tpaBreakpoint;
      cp->motor_output_limit = _model.config.output.motorLimit;
      cp->throttle_boost = 0;
      cp->throttle_boost_cutoff = 100;
      cp->anti_gravity_gain = 0;
      cp->anti_gravity_p_gain = 0;
      cp->anti_gravity_cutoff_hz = 100;
      cp->d_min_gain = 0;
      cp->d_min_advance = 0;
      cp->angle_limit = _model.config.angleLimit;
      cp->angle_earth_ref = 100;
      cp->horizon_limit_degrees = 135;
      cp->horizon_delay_ms = 500;
      cp->thrustLinearization = 0;
      cp->iterm_relax = _model.config.itermRelax;
      cp->iterm_relax_type = 1;
      cp->iterm_relax_cutoff = _model.config.itermRelaxCutoff;

      rcControlsConfigMutable()->deadband = _model.config.input.deadband;
      rcControlsConfigMutable()->yaw_deadband = _model.config.input.deadband;

      gyroConfigMutable()->gyro_hardware_lpf = _model.config.gyroDlpf;
      gyroConfigMutable()->gyro_lpf1_type = _model.config.gyroFilter.type;
      gyroConfigMutable()->gyro_lpf1_static_hz = _model.config.gyroFilter.freq;
      gyroConfigMutable()->gyro_lpf1_dyn_min_hz = _model.config.gyroDynLpfFilter.cutoff;
      gyroConfigMutable()->gyro_lpf1_dyn_max_hz = _model.config.gyroDynLpfFilter.freq;
      gyroConfigMutable()->gyro_lpf2_type = _model.config.gyroFilter2.type;
      gyroConfigMutable()->gyro_lpf2_static_hz = _model.config.gyroFilter2.freq;
      gyroConfigMutable()->gyro_soft_notch_cutoff_1 = _model.config.gyroNotch1Filter.cutoff;
      gyroConfigMutable()->gyro_soft_notch_hz_1 = _model.config.gyroNotch1Filter.freq;
      gyroConfigMutable()->gyro_soft_notch_cutoff_2 = _model.config.gyroNotch2Filter.cutoff;
      gyroConfigMutable()->gyro_soft_notch_hz_2 = _model.config.gyroNotch2Filter.freq;

      dynNotchConfigMutable()->dyn_notch_count = _model.config.dynamicFilter.width;
      dynNotchConfigMutable()->dyn_notch_q = _model.config.dynamicFilter.q;
      dynNotchConfigMutable()->dyn_notch_min_hz = _model.config.dynamicFilter.min_freq;
      dynNotchConfigMutable()->dyn_notch_max_hz = _model.config.dynamicFilter.max_freq;

      accelerometerConfigMutable()->acc_lpf_hz = _model.config.accelFilter.freq;
      accelerometerConfigMutable()->acc_hardware = _model.config.accelDev;
      barometerConfigMutable()->baro_hardware = _model.config.baroDev;
      compassConfigMutable()->mag_hardware = _model.config.magDev;

      motorConfigMutable()->dev.useUnsyncedPwm = _model.config.output.async;
      motorConfigMutable()->dev.motorPwmProtocol = _model.config.output.protocol;
      motorConfigMutable()->dev.motorPwmRate = _model.config.output.rate;
      motorConfigMutable()->mincommand = _model.config.output.minCommand;
      motorConfigMutable()->digitalIdleOffsetValue = _model.config.output.dshotIdle;
      motorConfigMutable()->minthrottle = _model.state.minThrottle;
      motorConfigMutable()->maxthrottle = _model.state.maxThrottle;

      gyroConfigMutable()->gyro_sync_denom = 1;
      pidConfigMutable()->pid_process_denom = _model.config.loopSync;

      if(_model.accelActive()) enabledSensors |= SENSOR_ACC;
      if(_model.magActive()) enabledSensors |= SENSOR_MAG;
      if(_model.baroActive()) enabledSensors |= SENSOR_BARO;

      gyro.sampleLooptime = _model.state.gyroTimer.interval;
      targetPidLooptime = _model.state.loopTimer.interval;
      activePidLoopDenom = _model.config.loopSync;

      if(_model.config.blackboxPdenom >= 0 && _model.config.blackboxPdenom <= 4)
      {
        blackboxConfigMutable()->sample_rate = _model.config.blackboxPdenom;
      }
      else
      {
        blackboxConfigMutable()->sample_rate = blackboxCalculateSampleRate(_model.config.blackboxPdenom);
      }
      blackboxConfigMutable()->device = _model.config.blackboxDev;
      blackboxConfigMutable()->fields_disabled_mask = _model.config.blackboxFieldsDisabledMask;
      blackboxConfigMutable()->mode = _model.config.blackboxMode;

      featureConfigMutable()->enabledFeatures = _model.config.featureMask;

      batteryConfigMutable()->currentMeterSource = (currentMeterSource_e)_model.config.ibatSource;
      batteryConfigMutable()->voltageMeterSource = (voltageMeterSource_e)_model.config.vbatSource;
      batteryConfigMutable()->vbatwarningcellvoltage = _model.config.vbatCellWarning;
      batteryConfigMutable()->vbatmaxcellvoltage = 420;
      batteryConfigMutable()->vbatmincellvoltage = 340;

      rxConfigMutable()->rcInterpolation = _model.config.input.interpolationMode;
      rxConfigMutable()->rcInterpolationInterval = _model.config.input.interpolationInterval;
      rxConfigMutable()->rssi_channel = _model.config.input.rssiChannel;
      rxConfigMutable()->airModeActivateThreshold = 40;
      rxConfigMutable()->serialrx_provider = _model.config.input.serialRxProvider;

      blackboxInit();

      return 1;
    }

    int update()
    {
      if(!_model.blackboxEnabled()) return 0;
      if(!blackboxSerial) return 0;
      Stats::Measure measure(_model.state.stats, COUNTER_BLACKBOX);

      uint32_t startTime = micros();
      updateArmed();
      updateMode();
      updateData();
      blackboxUpdate(_model.state.loopTimer.last);
      _buffer.flush();

      if(_model.config.debugMode == DEBUG_PIDLOOP)
      {
        _model.state.debug[5] = micros() - startTime;
      }

      return 1;
    }

  private:
    void updateData()
    {
      for(size_t i = 0; i < 3; i++)
      {
        gyro.gyroADCf[i] = degrees(_model.state.gyro[i]);
        gyro.gyroADC[i] = degrees(_model.state.gyroScaled[i]);
        pidData[i].P = _model.state.innerPid[i].pTerm * 1000.f;
        pidData[i].I = _model.state.innerPid[i].iTerm * 1000.f;
        pidData[i].D = _model.state.innerPid[i].dTerm * 1000.f;
        pidData[i].F = _model.state.innerPid[i].fTerm * 1000.f;
        rcCommand[i] = (_model.state.inputBuffer[i] - 1500) * (i == AXIS_YAW ? -1 : 1);
        if(_model.accelActive()) {
          acc.accADC[i] = _model.state.accel[i] * ACCEL_G_INV * acc.dev.acc_1G;
        }
        if(_model.magActive()) {
          mag.magADC[i] = _model.state.mag[i] * 1090;
        }
        if(_model.baroActive()) {
          baro.altitude = lrintf(_model.state.baroAltitude * 100.f); // cm
        }
      }
      rcCommand[AXIS_THRUST] = _model.state.inputBuffer[AXIS_THRUST];
      for(size_t i = 0; i < 4; i++)
      {
        motor[i] = Math::clamp(_model.state.outputUs[i], (int16_t)1000, (int16_t)2000);
        if(_model.state.digitalOutput)
        {
          motor[i] = PWM_TO_DSHOT(motor[i]);
        }
      }
      if(_model.config.debugMode != DEBUG_NONE && _model.config.debugMode != DEBUG_BLACKBOX_OUTPUT)
      {
        for(size_t i = 0; i < 8; i++)
        {
          debug[i] = _model.state.debug[i];
        }
      }
    }

    void updateArmed()
    {
      // log arming beep event
      static uint32_t beep = 0;
      if(beep != 0 && _model.state.loopTimer.last > beep)
      {
        setArmingBeepTimeMicros(_model.state.loopTimer.last);
        beep = 0;
      }

      // stop logging
      static uint32_t stop = 0;
      if(stop != 0 && _model.state.loopTimer.last > stop)
      {
        blackboxFinish();
        stop = 0;
      }

      bool armed = _model.isActive(MODE_ARMED);
      if(armed == ARMING_FLAG(ARMED)) return;
      if(armed)
      {
        ENABLE_ARMING_FLAG(ARMED);
        beep = _model.state.loopTimer.last + 200000; // schedule arming beep event ~200ms
      }
      else
      {
        DISABLE_ARMING_FLAG(ARMED);
        flightLogEventData_t eventData;
        eventData.disarm.reason = _model.state.disarmReason;
        blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, &eventData);
        stop = _model.state.loopTimer.last + 500000; // schedule stop in 500ms
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
    BlackboxBuffer _buffer;
};

}
#endif
