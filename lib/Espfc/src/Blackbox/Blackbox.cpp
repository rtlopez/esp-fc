#include "Blackbox.h"
#include "Hardware.h"
#include "EscDriver.h"
#include "Math/Utils.h"
#include "BlackboxBridge.h"

namespace Espfc {

namespace Blackbox {

static void updateModeFlag(boxBitmask_t *mask, boxId_e id, bool value)
{
  if(value) bitArraySet(mask, id);
  else bitArrayClr(mask, id);
}

Blackbox::Blackbox(Model& model): _model(model) {}

int Blackbox::begin()
{
  initBlackboxModel(&_model);

#ifdef USE_FLASHFS
  int res = flashfsInit();
  _model.logger.info().log(F("FLASHFS")).log(res).logln(flashfsGetOffset());
#endif

  if(!_model.blackboxEnabled()) return 0;

  if(_model.config.blackboxDev == 3)
  {
    _serial = _model.getSerialStream(SERIAL_FUNCTION_BLACKBOX);
    if(!_serial) return 0;

    _buffer.wrap(_serial);
    serialDeviceInit(&_buffer, 0);
    //serialDeviceInit(_serial, 0);
  }

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
  for(size_t i = 0; i < FC_PID_ITEM_COUNT; i++)
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
  cp->itermWindupPointPercent = 80;
  cp->antiGravityMode = 0;
  cp->pidSumLimit = 660;
  cp->pidSumLimitYaw = 660;
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
  cp->dterm_lpf1_dyn_expo = 5;
  cp->tpa_low_rate = 20;
  cp->tpa_low_breakpoint = 1050;
  cp->tpa_low_always = 0;
  cp->ez_landing_threshold = 25;
  cp->ez_landing_limit = 5;

  rcControlsConfigMutable()->deadband = _model.config.input.deadband;
  rcControlsConfigMutable()->yaw_deadband = _model.config.input.deadband;

  gyroConfigMutable()->gyro_hardware_lpf = _model.config.gyroDlpf;
  gyroConfigMutable()->gyro_lpf1_type = _model.config.gyroFilter.type;
  gyroConfigMutable()->gyro_lpf1_static_hz = _model.config.gyroFilter.freq;
  gyroConfigMutable()->gyro_lpf1_dyn_min_hz = _model.config.gyroDynLpfFilter.cutoff;
  gyroConfigMutable()->gyro_lpf1_dyn_max_hz = _model.config.gyroDynLpfFilter.freq;
  gyroConfigMutable()->gyro_lpf1_dyn_expo = 5;
  gyroConfigMutable()->gyro_lpf2_type = _model.config.gyroFilter2.type;
  gyroConfigMutable()->gyro_lpf2_static_hz = _model.config.gyroFilter2.freq;
  gyroConfigMutable()->gyro_soft_notch_cutoff_1 = _model.config.gyroNotch1Filter.cutoff;
  gyroConfigMutable()->gyro_soft_notch_hz_1 = _model.config.gyroNotch1Filter.freq;
  gyroConfigMutable()->gyro_soft_notch_cutoff_2 = _model.config.gyroNotch2Filter.cutoff;
  gyroConfigMutable()->gyro_soft_notch_hz_2 = _model.config.gyroNotch2Filter.freq;
  gyroConfigMutable()->gyro_sync_denom = 1;

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
  motorConfigMutable()->dev.useDshotTelemetry = _model.config.output.dshotTelemetry;
  motorConfigMutable()->motorPoleCount = _model.config.output.motorPoles;

  pidConfigMutable()->pid_process_denom = _model.config.loopSync;

  mixerConfigMutable()->mixer_type = 0;

  if(_model.accelActive()) sensorsSet(SENSOR_ACC);
  if(_model.magActive()) sensorsSet(SENSOR_MAG);
  if(_model.baroActive()) sensorsSet(SENSOR_BARO);

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

  rpmFilterConfigMutable()->rpm_filter_harmonics = _model.config.rpmFilterHarmonics;
  rpmFilterConfigMutable()->rpm_filter_q = _model.config.rpmFilterQ;
  rpmFilterConfigMutable()->rpm_filter_min_hz = _model.config.rpmFilterMinFreq;
  rpmFilterConfigMutable()->rpm_filter_fade_range_hz = _model.config.rpmFilterFade;
  rpmFilterConfigMutable()->rpm_filter_lpf_hz = _model.config.rpmFilterFreqLpf;
  rpmFilterConfigMutable()->rpm_filter_weights[0] = _model.config.rpmFilterWeights[0];
  rpmFilterConfigMutable()->rpm_filter_weights[1] = _model.config.rpmFilterWeights[1];
  rpmFilterConfigMutable()->rpm_filter_weights[2] = _model.config.rpmFilterWeights[2];

  updateModeFlag(&rcModeActivationPresent, BOXARM, _model.state.modeMaskPresent & 1 << MODE_ARMED);
  updateModeFlag(&rcModeActivationPresent, BOXANGLE, _model.state.modeMaskPresent & 1 << MODE_ANGLE);
  updateModeFlag(&rcModeActivationPresent, BOXAIRMODE, _model.state.modeMaskPresent & 1 << MODE_AIRMODE);
  updateModeFlag(&rcModeActivationPresent, BOXFAILSAFE, _model.state.modeMaskPresent & 1 << MODE_FAILSAFE);
  updateModeFlag(&rcModeActivationPresent, BOXBLACKBOX, _model.state.modeMaskPresent & 1 << MODE_BLACKBOX);
  updateModeFlag(&rcModeActivationPresent, BOXBLACKBOXERASE, _model.state.modeMaskPresent & 1 << MODE_BLACKBOX_ERASE);

  blackboxInit();

  return 1;
}

int FAST_CODE_ATTR Blackbox::update()
{
  if(!_model.blackboxEnabled()) return 0;
  if(_model.config.blackboxDev == 3 && !_serial) return 0;

  Stats::Measure measure(_model.state.stats, COUNTER_BLACKBOX);

  uint32_t startTime = micros();
  updateArmed();
  updateMode();
  if(blackboxShouldLogIFrame() || blackboxShouldLogPFrame())
  {
    updateData();
  }
  //PIN_DEBUG(HIGH);
  blackboxUpdate(_model.state.loopTimer.last);
  if(_model.config.blackboxDev == 3)
  {
    _buffer.flush();
  }
  //PIN_DEBUG(LOW);

  if(_model.config.debugMode == DEBUG_PIDLOOP)
  {
    _model.state.debug[5] = micros() - startTime;
  }

  return 1;
}

void FAST_CODE_ATTR Blackbox::updateData()
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

void FAST_CODE_ATTR Blackbox::updateArmed()
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

void FAST_CODE_ATTR Blackbox::updateMode()
{
  updateModeFlag(&rcModeActivationMask, BOXARM, _model.isSwitchActive(MODE_ARMED));
  updateModeFlag(&rcModeActivationMask, BOXANGLE, _model.isSwitchActive(MODE_ANGLE));
  updateModeFlag(&rcModeActivationMask, BOXAIRMODE, _model.isSwitchActive(MODE_AIRMODE));
  updateModeFlag(&rcModeActivationMask, BOXFAILSAFE, _model.isSwitchActive(MODE_FAILSAFE));
  updateModeFlag(&rcModeActivationMask, BOXBLACKBOX, _model.isSwitchActive(MODE_BLACKBOX));
  updateModeFlag(&rcModeActivationMask, BOXBLACKBOXERASE, _model.isSwitchActive(MODE_BLACKBOX_ERASE));
}

}

}
