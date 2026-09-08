#include "Blackbox.h"
#include "BlackboxBridge.h"
#include "Utils/Math.hpp"

namespace Espfc::Blackbox {

static void updateModeFlag(boxBitmask_t* mask, boxId_e id, bool value)
{
  if (value)
  {
    bitArraySet(mask, id);
  }
  else
  {
    bitArrayClr(mask, id);
  }
}

Blackbox::Blackbox(Model& model): _model(model) {}

int Blackbox::begin()
{
  initBlackboxModel(&_model);

#ifdef USE_FLASHFS
  int res = flashfsInit();
  _model.logger.info().log("FLASHFS").log(res).logln(flashfsGetOffset());
#endif

  if (!_model.blackboxEnabled()) return 0;

  if (_model.config.blackbox.dev == BLACKBOX_DEV_SERIAL)
  {
    _serial = _model.getSerialStream(SERIAL_FUNCTION_BLACKBOX);
    if (!_serial) return 0;

    _buffer.wrap(_serial);
    serialDeviceInit(&_buffer, 0);
    // serialDeviceInit(_serial, 0);
  }

  gyro.sampleLooptime = _model.state.gyro.timer.interval;
  targetPidLooptime = _model.state.loopTimer.interval;
  activePidLoopDenom = _model.config.loopSync;

  systemConfigMutable()->pidProfileIndex = 0;
  systemConfigMutable()->activeRateProfile = 0;
  systemConfigMutable()->activeBatteryProfile = 0;
  systemConfigMutable()->debug_mode = debugMode = _model.config.debug.mode;

  armingConfigMutable()->gyro_cal_on_first_arm = 0;

  auto* pc = pilotConfigMutable();
  std::fill_n(pc->craftName, MAX_NAME_LENGTH + 1, 0);
  std::copy_n(_model.config.modelName, std::min<size_t>(MODEL_NAME_LEN, MAX_NAME_LENGTH), pc->craftName);

  auto* rp = controlRateProfilesMutable(systemConfig()->activeRateProfile);
  for (int i = 0; i < AXIS_COUNT_RPY; i++)
  {
    rp->rcRates[i] = _model.config.input.rate[i];
    rp->rcExpo[i] = _model.config.input.expo[i];
    rp->rates[i] = _model.config.input.superRate[i];
    rp->rate_limit[i] = _model.config.input.rateLimit[i];
  }
  rp->thrMid8 = 50;
  rp->thrExpo8 = 0;
  rp->thrHover8 = 50;
  rp->dynThrPID = _model.config.controller.tpaScale;
  rp->tpa_breakpoint = _model.config.controller.tpaBreakpoint;
  rp->rates_type = _model.config.input.rateType;
  rp->throttle_limit_type = _model.config.output.throttleLimitType;
  rp->throttle_limit_percent = _model.config.output.throttleLimitPercent;

  auto* cp = currentPidProfile = pidProfilesMutable(systemConfig()->pidProfileIndex);
  for (size_t i = 0; i < PID_ITEM_COUNT; i++)
  {
    cp->pid[i].P = _model.config.pid[i].P;
    cp->pid[i].I = _model.config.pid[i].I;
    cp->pid[i].D = _model.config.pid[i].D;
    cp->pid[i].F = _model.config.pid[i].F;
    if (i <= AXIS_YAW)
    {
      cp->d_max[i] = _model.config.pid[i].D;
    }
  }
  cp->pidAtMinThrottle = 1;
  cp->dterm_lpf1_type = _model.config.dterm.filter.type;
  cp->dterm_lpf1_static_hz = _model.config.dterm.filter.freq;
  cp->dterm_lpf1_dyn_min_hz = _model.config.dterm.dynLpfFilter.cutoff;
  cp->dterm_lpf1_dyn_max_hz = _model.config.dterm.dynLpfFilter.freq;
  cp->dterm_lpf2_type = _model.config.dterm.filter2.type;
  cp->dterm_lpf2_static_hz = _model.config.dterm.filter2.freq;
  cp->dterm_notch_hz = _model.config.dterm.notchFilter.freq;
  cp->dterm_notch_cutoff = _model.config.dterm.notchFilter.cutoff;
  cp->yaw_lowpass_hz = _model.config.yaw.filter.freq;
  cp->itermWindup = 80;
  cp->antiGravityMode = 0;
  cp->pidSumLimit = 660;
  cp->pidSumLimitYaw = 660;
  cp->ff_boost = 0;
  cp->feedForwardTransition = 0;
  cp->tpa_mode = _model.config.controller.tpaMode;
  cp->tpa_rate = _model.config.controller.tpaScale;
  cp->tpa_breakpoint = _model.config.controller.tpaBreakpoint;
  cp->motor_output_limit = _model.config.output.motorLimit;
  cp->throttle_boost = 0;
  cp->throttle_boost_cutoff = 100;
  cp->anti_gravity_gain = 0;
  cp->anti_gravity_p_gain = 0;
  cp->anti_gravity_cutoff_hz = 100;
  cp->d_max_gain = 0;
  cp->d_max_advance = 0;
  cp->angle_limit = _model.config.level.angleLimit;
  cp->angle_earth_ref = 100;
  cp->horizon_limit_degrees = 135;
  cp->horizon_delay_ms = 500;
  cp->thrustLinearization = 0;
  cp->iterm_relax = _model.config.iterm.relax;
  cp->iterm_relax_type = 1;
  cp->iterm_relax_cutoff = _model.config.iterm.relaxCutoff;
  cp->dterm_lpf1_dyn_expo = 5;
  cp->tpa_low_rate = 20;
  cp->tpa_low_breakpoint = 1050;
  cp->tpa_low_always = 0;
  cp->ez_landing_threshold = 25;
  cp->ez_landing_limit = 5;
  cp->ez_landing_speed = 50;
  cp->landing_disarm_threshold = 0;
  cp->yawRateAccelLimit = 0;
  cp->rateAccelLimit = 0;

  auto* bp = currentBatteryProfile = batteryProfilesMutable(systemConfig()->activeBatteryProfile);
  bp->vbatmaxcellvoltage = 430;
  bp->vbatmincellvoltage = 330;
  bp->vbatwarningcellvoltage = _model.config.vbat.cellWarning;

  rcControlsConfigMutable()->deadband = _model.config.input.deadband;
  rcControlsConfigMutable()->yaw_deadband = _model.config.input.deadband;

  gyroConfigMutable()->gyro_hardware_lpf = _model.config.gyro.dlpf;
  gyroConfigMutable()->gyro_lpf1_type = _model.config.gyro.filter.type;
  gyroConfigMutable()->gyro_lpf1_static_hz = _model.config.gyro.filter.freq;
  gyroConfigMutable()->gyro_lpf1_dyn_min_hz = _model.config.gyro.dynLpfFilter.cutoff;
  gyroConfigMutable()->gyro_lpf1_dyn_max_hz = _model.config.gyro.dynLpfFilter.freq;
  gyroConfigMutable()->gyro_lpf1_dyn_expo = 5;
  gyroConfigMutable()->gyro_lpf2_type = _model.config.gyro.filter2.type;
  gyroConfigMutable()->gyro_lpf2_static_hz = _model.config.gyro.filter2.freq;
  gyroConfigMutable()->gyro_soft_notch_cutoff_1 = _model.config.gyro.notch1Filter.cutoff;
  gyroConfigMutable()->gyro_soft_notch_hz_1 = _model.config.gyro.notch1Filter.freq;
  gyroConfigMutable()->gyro_soft_notch_cutoff_2 = _model.config.gyro.notch2Filter.cutoff;
  gyroConfigMutable()->gyro_soft_notch_hz_2 = _model.config.gyro.notch2Filter.freq;
  gyroConfigMutable()->gyro_sync_denom = 1;
  gyroConfigMutable()->gyro_filter_debug_axis = _model.config.debug.axis;
  gyroConfigMutable()->gyro_enabled_bitmask = 1;

  dynNotchConfigMutable()->dyn_notch_count = _model.config.gyro.dynamicFilter.count;
  dynNotchConfigMutable()->dyn_notch_q = _model.config.gyro.dynamicFilter.q;
  dynNotchConfigMutable()->dyn_notch_min_hz = _model.config.gyro.dynamicFilter.min_freq;
  dynNotchConfigMutable()->dyn_notch_max_hz = _model.config.gyro.dynamicFilter.max_freq;

  accelerometerConfigMutable()->acc_lpf_hz = _model.config.accel.filter.freq;
  accelerometerConfigMutable()->acc_hardware = _model.config.accel.dev;
  barometerConfigMutable()->baro_hardware = _model.config.baro.dev;
  barometerConfigMutable()->baroTempDriftCmPer10C = 0;
  compassConfigMutable()->mag_hardware = _model.config.mag.dev;

  motorConfigMutable()->dev.useContinuousUpdate = _model.config.output.async;
  motorConfigMutable()->dev.motorProtocol = _model.config.output.protocol;
  motorConfigMutable()->dev.motorPwmRate = _model.config.output.rate;
  motorConfigMutable()->mincommand = _model.config.output.minCommand;
  motorConfigMutable()->motorIdle = _model.config.output.motorIdle;
  motorConfigMutable()->maxthrottle = _model.state.mixer.maxThrottle;
  motorConfigMutable()->motorPoleCount = _model.config.output.motorPoles;
  motorConfigMutable()->kv = 1960;
  useDshotTelemetry = _model.config.output.dshotTelemetry;

  pidConfigMutable()->pid_process_denom = _model.config.loopSync;

  mixerConfigMutable()->mixer_type = 0;

  if (_model.accelActive()) sensorsSet(SENSOR_ACC);
  if (_model.magActive()) sensorsSet(SENSOR_MAG);
  if (_model.baroActive()) sensorsSet(SENSOR_BARO);

  blackboxConfigMutable()->sample_rate = _model.config.blackbox.pDenom;
  blackboxConfigMutable()->device = _model.config.blackbox.dev;
  blackboxConfigMutable()->fields_disabled_mask = ~_model.config.blackbox.fieldsMask;
  blackboxConfigMutable()->mode = _model.config.blackbox.mode;
  blackboxConfigMutable()->high_resolution = 0;

  featureConfigMutable()->enabledFeatures = _model.config.featureMask;

  batteryConfigMutable()->currentMeterSource = (currentMeterSource_e)_model.config.ibat.source;
  batteryConfigMutable()->voltageMeterSource = (voltageMeterSource_e)_model.config.vbat.source;
  voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = _model.config.vbat.scale;
  currentSensorADCConfigMutable()->scale = _model.config.ibat.scale;
  currentSensorADCConfigMutable()->offset = _model.config.ibat.offset;

  rxConfigMutable()->rssi_channel = _model.config.input.rssiChannel;
  rxConfigMutable()->airModeActivateThreshold = _model.config.input.airModeActivateThreshold;
  rxConfigMutable()->serialrx_provider = _model.config.input.serialRxProvider;

  rpmFilterConfigMutable()->rpm_filter_harmonics = _model.config.gyro.rpmFilter.harmonics;
  rpmFilterConfigMutable()->rpm_filter_q = _model.config.gyro.rpmFilter.q;
  rpmFilterConfigMutable()->rpm_filter_min_hz = _model.config.gyro.rpmFilter.minFreq;
  rpmFilterConfigMutable()->rpm_filter_fade_range_hz = _model.config.gyro.rpmFilter.fade;
  rpmFilterConfigMutable()->rpm_filter_lpf_hz = _model.config.gyro.rpmFilter.freqLpf;
  rpmFilterConfigMutable()->rpm_filter_weights[0] = _model.config.gyro.rpmFilter.weights[0];
  rpmFilterConfigMutable()->rpm_filter_weights[1] = _model.config.gyro.rpmFilter.weights[1];
  rpmFilterConfigMutable()->rpm_filter_weights[2] = _model.config.gyro.rpmFilter.weights[2];

  gpsConfigMutable()->provider = 1; // ubx
  gpsConfigMutable()->gps_set_home_point_once = false;
  gpsConfigMutable()->gps_use_3d_speed = false;

  positionConfigMutable()->altitude_source = 0;
  positionConfigMutable()->altitude_prefer_baro = 50;
  positionConfigMutable()->altitude_lpf = 300;
  positionConfigMutable()->altitude_d_lpf = 300;

  autopilotConfigMutable()->landingAltitudeM = 4;
  autopilotConfigMutable()->hoverThrottle = 1275;
  autopilotConfigMutable()->throttleMin = 1100;
  autopilotConfigMutable()->throttleMax = 1900;

  autopilotConfigMutable()->altitudeP = 30;
  autopilotConfigMutable()->altitudeI = 30;
  autopilotConfigMutable()->altitudeD = 30;
  autopilotConfigMutable()->altitudeA = 30;
  autopilotConfigMutable()->altitudeF = 30;

  autopilotConfigMutable()->positionP = 30;
  autopilotConfigMutable()->positionI = 30;
  autopilotConfigMutable()->positionD = 30;
  autopilotConfigMutable()->positionA = 30;
  autopilotConfigMutable()->positionF = 30;
  autopilotConfigMutable()->positionCutoff = 30;
  autopilotConfigMutable()->stopThreshold = 10;
  autopilotConfigMutable()->maxAngle = 50;

  updateModeFlag(&rcModeActivationPresent, BOXARM, _model.state.mode.maskPresent & 1 << MODE_ARMED);
  updateModeFlag(&rcModeActivationPresent, BOXANGLE, _model.state.mode.maskPresent & 1 << MODE_ANGLE);
  updateModeFlag(&rcModeActivationPresent, BOXAIRMODE, _model.state.mode.maskPresent & 1 << MODE_AIRMODE);
  updateModeFlag(&rcModeActivationPresent, BOXFAILSAFE, _model.state.mode.maskPresent & 1 << MODE_FAILSAFE);
  updateModeFlag(&rcModeActivationPresent, BOXBLACKBOX, _model.state.mode.maskPresent & 1 << MODE_BLACKBOX);
  updateModeFlag(&rcModeActivationPresent, BOXBLACKBOXERASE, _model.state.mode.maskPresent & 1 << MODE_BLACKBOX_ERASE);

  blackboxInit();

  return 1;
}

int FAST_CODE_ATTR Blackbox::update()
{
  if (!_model.blackboxEnabled()) return 0;
  if (_model.config.blackbox.dev == BLACKBOX_DEV_SERIAL && !_serial) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BLACKBOX);

  uint32_t startTime = micros();
  updateArmed();
  updateMode();
  if (blackboxShouldLogIFrame() || blackboxShouldLogPFrame())
  {
    updateData();
  }
  // PIN_DEBUG(HIGH);
  blackboxUpdate(_model.state.loopTimer.last);
  if (_model.config.blackbox.dev == BLACKBOX_DEV_SERIAL)
  {
    _buffer.flush();
  }
  // PIN_DEBUG(LOW);

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[5] = micros() - startTime;
  }

  return 1;
}

void FAST_CODE_ATTR Blackbox::updateData()
{
  auto accel = _model.state.accel.adc.fetch();
  for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
    gyro.gyroADCf[i] = Utils::toDeg(_model.state.gyro.adc[i]);
    gyro.gyroADC[i] = Utils::toDeg(_model.state.gyro.scaled[i]);
    pidData[i].P = _model.state.innerPid[i].pTerm * 1000.f;
    pidData[i].I = _model.state.innerPid[i].iTerm * 1000.f;
    pidData[i].D = _model.state.innerPid[i].dTerm * 1000.f;
    pidData[i].F = _model.state.innerPid[i].fTerm * 1000.f;
    rcCommand[i] = (_model.state.input.buffer[i] - 1500) * (i == AXIS_YAW ? -1 : 1);
    acc.accADC.v[i] = _model.accelActive() ? accel[i] * ACCEL_G_INV * acc.dev.acc_1G : 0.f;
    mag.magADC.v[i] = _model.magActive() ? _model.state.mag.adc[i] * 1090 : 0.f;
  }
  baro.altitude = _model.baroActive() ? lrintf(_model.state.baro.altitudeGround * 100.f) : 0; // cm
  rcCommand[AXIS_THRUST] = _model.state.input.buffer[AXIS_THRUST];

  const auto& q = _model.state.attitude.quaternion;
  imuAttitudeQuaternion.w = q.w;
  imuAttitudeQuaternion.x = q.x;
  imuAttitudeQuaternion.y = q.y;
  imuAttitudeQuaternion.z = q.z;

  const size_t motorCount = std::min<size_t>(getMotorCount(), OUTPUT_CHANNELS);
  for (size_t i = 0; i < MAX_SUPPORTED_MOTORS; i++)
  {
    if (i >= motorCount)
    {
      motor[i] = 0;
      continue;
    }
    const auto mv = std::clamp<int16_t>(_model.state.output.us[i], 1000, 2000);
    motor[i] = _model.state.mixer.digitalOutput ? PWM_TO_DSHOT(mv) : mv;
  }
  if (_model.config.debug.mode != DEBUG_NONE && _model.config.debug.mode != DEBUG_BLACKBOX_OUTPUT)
  {
    for (size_t i = 0; i < DEBUG_VALUE_COUNT; i++)
    {
      debug[i] = _model.state.debug[i];
    }
  }

  if (_lastGpsTs == 0 || _lastGpsTs != _model.state.gps.lastMsgTs)
  {
    GPS_home_llh.lat = _model.state.gps.location.home.lat;
    GPS_home_llh.lon = _model.state.gps.location.home.lon;
    GPS_home_llh.altCm = (_model.state.gps.location.home.height + 5) / 10;
    gpsSol.llh.lat = _model.state.gps.location.raw.lat;
    gpsSol.llh.lon = _model.state.gps.location.raw.lon;
    gpsSol.llh.altCm = (_model.state.gps.location.raw.height + 5) / 10;                                       // 0.1 m
    gpsSol.groundSpeed = (_model.state.gps.velocity.raw.groundSpeed + 5) / 10;                                // cm/s
    gpsSol.groundCourse = (_model.state.gps.velocity.raw.heading + 5000) / 10000;                             // 0.1 deg
    gpsSol.speed3d = (_model.state.gps.velocity.raw.speed3d + 5) / 10;                                        // cm/s
    gpsSol.velned.velN = std::clamp<int32_t>(_model.state.gps.velocity.raw.north / 10, INT16_MIN, INT16_MAX); // cm/s
    gpsSol.velned.velE = std::clamp<int32_t>(_model.state.gps.velocity.raw.east / 10, INT16_MIN, INT16_MAX);  // cm/s
    gpsSol.velned.velD = std::clamp<int32_t>(_model.state.gps.velocity.raw.down / 10, INT16_MIN, INT16_MAX);  // cm/s
    gpsSol.numSat = _model.state.gps.numSats;
    gpsSol.hdop = _model.state.gps.accuracy.pDop; // (1e2)
    gpsSol.time = _model.state.gps.time;

    const GpsDateTime& dt = _model.state.gps.dateTime;
    gpsSol.dateTime.year = dt.year;
    gpsSol.dateTime.month = dt.month;
    gpsSol.dateTime.day = dt.day;
    gpsSol.dateTime.hour = dt.hour;
    gpsSol.dateTime.min = dt.minute;
    gpsSol.dateTime.sec = dt.second;
    gpsSol.dateTime.millis = dt.msec;
    gpsSol.dateTime.valid = dt.year >= 1980 && dt.month >= 1 && dt.month <= 12 && dt.day >= 1 && dt.day <= 31;
    _lastGpsTs = _model.state.gps.lastMsgTs;
  }
}

void FAST_CODE_ATTR Blackbox::updateArmed()
{
  // log arming beep event
  static uint32_t beep = 0;
  if (beep != 0 && _model.state.loopTimer.last > beep)
  {
    setArmingBeepTimeMicros(_model.state.loopTimer.last);
    beep = 0;
  }

  // stop logging
  static uint32_t stop = 0;
  if (stop != 0 && _model.state.loopTimer.last > stop)
  {
    blackboxFinish();
    stop = 0;
  }

  bool armed = _model.isModeActive(MODE_ARMED);
  if (armed == ARMING_FLAG(ARMED)) return;
  if (armed)
  {
    ENABLE_ARMING_FLAG(ARMED);
    beep = _model.state.loopTimer.last + 200000; // schedule arming beep event ~200ms
  }
  else
  {
    DISABLE_ARMING_FLAG(ARMED);
    flightLogEventData_t eventData;
    eventData.disarm.reason = _model.state.mode.disarmReason;
    blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, &eventData);
    stop = _model.state.loopTimer.last + 500000; // schedule stop in 500ms
  }
}

void FAST_CODE_ATTR Blackbox::updateMode()
{
  updateModeFlag(&rcModeActivationMask, BOXARM, _model.isSwitchActive(MODE_ARMED));
  updateModeFlag(&rcModeActivationMask, BOXANGLE, _model.isSwitchActive(MODE_ANGLE));
  updateModeFlag(&rcModeActivationMask, BOXALTHOLD, _model.isSwitchActive(MODE_ALTHOLD));
  updateModeFlag(&rcModeActivationMask, BOXFAILSAFE, _model.isSwitchActive(MODE_FAILSAFE));
  updateModeFlag(&rcModeActivationMask, BOXBLACKBOX, _model.isSwitchActive(MODE_BLACKBOX));
  updateModeFlag(&rcModeActivationMask, BOXAIRMODE, _model.isSwitchActive(MODE_AIRMODE));
  updateModeFlag(&rcModeActivationMask, BOXBLACKBOXERASE, _model.isSwitchActive(MODE_BLACKBOX_ERASE));
}

} // namespace Espfc::Blackbox
