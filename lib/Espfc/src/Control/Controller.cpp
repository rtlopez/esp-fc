#include "Control/Controller.h"
#include "Utils/Math.hpp"
#include <algorithm>

namespace Espfc::Control {

Controller::Controller(Model& model): _model(model) {}

int Controller::begin()
{
  _rates.begin(_model.config.input);
  _speedFilter.begin(FilterConfig(FILTER_BIQUAD, 10), _model.state.loopTimer.rate);

  beginInnerLoop(AXIS_ROLL);
  beginInnerLoop(AXIS_PITCH);
  beginInnerLoop(AXIS_YAW);
  beginOuterLoop(AXIS_ROLL);
  beginOuterLoop(AXIS_PITCH);
  beginAltHold();

  return 1;
}

int FAST_CODE_ATTR Controller::update()
{
  uint32_t startTime = 0;
  if(_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    startTime = micros();
    _model.state.debug[0] = startTime - _model.state.loopTimer.last;
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_OUTER_PID);
    resetIterm();
    switch(_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL:
        outerLoopRobot();
        break;

      default:
        outerLoop();
        break;
    }
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_INNER_PID);
    switch(_model.config.mixer.type)
    {
      case FC_MIXER_GIMBAL:
        innerLoopRobot();
        break;

      default:
        innerLoop();
        break;
    }
  }

  if(_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[2] = micros() - startTime;
  }

  return 1;
}

void Controller::outerLoopRobot()
{
  const float speedScale = 2.f;
  const float gyroScale = 0.1f;
  const float speed = _speedFilter.update(_model.state.output.ch[AXIS_PITCH] * speedScale + _model.state.gyro.adc[AXIS_PITCH] * gyroScale);
  float angle = 0;

  if(true || _model.isModeActive(MODE_ANGLE))
  {
    angle = _model.state.input.ch[AXIS_PITCH] * Utils::toRad(_model.config.level.angleLimit);
  }
  else
  {
    angle = _model.state.outerPid[AXIS_PITCH].update(_model.state.input.ch[AXIS_PITCH], speed) * Utils::toRad(_model.config.level.rateLimit);
  }
  _model.state.setpoint.angle.set(AXIS_PITCH, angle);
  _model.state.setpoint.rate[AXIS_YAW] = _model.state.input.ch[AXIS_YAW] * Utils::toRad(_model.config.level.rateLimit);

  if(_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[0] = speed * 1000;
    _model.state.debug[1] = lrintf(Utils::toDeg(angle) * 10);
  }
}

void Controller::innerLoopRobot()
{
  //VectorFloat v(0.f, 0.f, 1.f);
  //v.rotate(_model.state.attitude.quaternion);
  //const float angle = acos(v.z);
  const float angle = std::max(abs(_model.state.attitude.euler[AXIS_PITCH]), abs(_model.state.attitude.euler[AXIS_ROLL]));

  const bool stabilize = angle < Utils::toRad(_model.config.level.angleLimit);
  if(stabilize)
  {
    _model.state.output.ch[AXIS_PITCH] = _model.state.innerPid[AXIS_PITCH].update(_model.state.setpoint.angle[AXIS_PITCH], _model.state.attitude.euler[AXIS_PITCH]);
    _model.state.output.ch[AXIS_YAW]   = _model.state.innerPid[AXIS_YAW].update(_model.state.setpoint.rate[AXIS_YAW], _model.state.gyro.adc[AXIS_YAW]);
  }
  else
  {
    resetIterm();
    _model.state.output.ch[AXIS_PITCH] = 0.f;
    _model.state.output.ch[AXIS_YAW] = 0.f;
  }

  if(_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[2] = lrintf(Utils::toDeg(_model.state.attitude.euler[AXIS_PITCH]) * 10);
    _model.state.debug[3] = lrintf(_model.state.output.ch[AXIS_PITCH] * 1000);
  }
}

void FAST_CODE_ATTR Controller::outerLoop()
{
  // Roll/Pitch rates control
  if(_model.isModeActive(MODE_ANGLE))
  {
    for(size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      const float angleSetpoint = Utils::toRad(_model.config.level.angleLimit) * _model.state.input.ch[i];
      _model.state.setpoint.rate[i] = _model.state.outerPid[i].update(angleSetpoint, _model.state.attitude.euler[i]);
      // disable fterm in angle mode
      _model.state.innerPid[i].fScale = 0.f;
    }
  }
  else
  {
    for(size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      _model.state.setpoint.rate[i] = calculateSetpointRate(i, _model.state.input.ch[i]);
    }
  }

  // Yaw rates control
  _model.state.setpoint.rate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input.ch[AXIS_YAW]);

  // thrust control
  if(_model.isModeActive(MODE_ALTHOLD))
  {
    _model.state.setpoint.rate[AXIS_THRUST] = calcualteAltHoldSetpoint();
  }
  else
  {
    _model.state.setpoint.rate[AXIS_THRUST] = _model.state.input.ch[AXIS_THRUST];
  }

  // debug
  if(_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    for(size_t i = 0; i < AXIS_COUNT_RPY; ++i)
    {
      _model.state.debug[i] = lrintf(Utils::toDeg(_model.state.setpoint.rate[i]));
    }
  }
}

void FAST_CODE_ATTR Controller::innerLoop()
{
  // Roll/Pitch/Yaw rates control
  const float tpaFactor = getTpaFactor();
  for(size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    _model.state.output.ch[i] = _model.state.innerPid[i].update(_model.state.setpoint.rate[i], _model.state.gyro.adc[i]) * tpaFactor;
  }

  // thrust control
  if(_model.isModeActive(MODE_ALTHOLD))
  {
    _model.state.output.ch[AXIS_THRUST] = _model.state.innerPid[AXIS_THRUST].update(_model.state.setpoint.rate[AXIS_THRUST], _model.state.altitude.vario);
  }
  else
  {
    _model.state.innerPid[AXIS_THRUST].update(0, _model.state.altitude.vario);
    _model.state.innerPid[AXIS_THRUST].iTerm = _model.state.input.ch[AXIS_THRUST]; // follow iTerm from rc input for smooth mid-air transition
    _model.state.output.ch[AXIS_THRUST] = _model.state.setpoint.rate[AXIS_THRUST];
  }

  if(_model.config.debug.mode == DEBUG_STACK)
  {
    _model.state.debug[0] = std::clamp(lrintf(_model.state.setpoint.rate[AXIS_THRUST] * 1000.0f), -3000l, 3000l); // stack hi mem
    _model.state.debug[1] = std::clamp(lrintf(_model.state.altitude.vario * 100.0f), -30000l, 30000l);            // stack lo mem
    _model.state.debug[2] = std::clamp(lrintf(_model.state.altitude.height * 100.0f), -30000l, 30000l);           // stack curr
    _model.state.debug[3] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].error * 1000.0f), -30000l, 30000l); // stack p
    _model.state.debug[4] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].pTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[5] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].iTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[6] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].dTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[7] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].fTerm * 1000.0f), -3000l, 3000l);
  }

  // debug
  if(_model.config.debug.mode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(_model.state.innerPid[0].itermRelaxBase));
    _model.state.debug[1] = lrintf(_model.state.innerPid[0].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Utils::toDeg(_model.state.innerPid[0].iTermError));
    _model.state.debug[3] = lrintf(_model.state.innerPid[0].iTerm * 1000.0f);
  }
}

float Controller::calcualteAltHoldSetpoint() const
{
  float thrust = _model.state.input.ch[AXIS_THRUST];

  //if(_model.isThrottleLow()) thrust = 0.0f; // stick below min check, no command

  thrust = Utils::deadband(thrust, 0.1f); // +/- 12.5% deadband

  return thrust * 2.0f; // climb/descend rate factor 2 m/s
}

float Controller::getTpaFactor() const
{
  if(_model.config.controller.tpaScale == 0) return 1.f;
  float t = Utils::clamp(_model.state.input.us[AXIS_THRUST], (float)_model.config.controller.tpaBreakpoint, 2000.f);
  return Utils::map(t, (float)_model.config.controller.tpaBreakpoint, 2000.f, 1.f, 1.f - ((float)_model.config.controller.tpaScale * 0.01f));
}

void Controller::resetIterm()
{
  if(!_model.isModeActive(MODE_ARMED)   // when not armed
    || (!_model.isAirModeActive() && _model.config.iterm.lowThrottleZeroIterm && _model.isThrottleLow()) // on low throttle (not in air mode)
  )
  {
    for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
    {
      _model.state.innerPid[i].resetIterm();
      _model.state.outerPid[i].resetIterm();
    }
  }
  if(!_model.isModeActive(MODE_ARMED))
  {
    //_model.state.innerPid[AXIS_THRUST].resetIterm();
  }
}

float Controller::calculateSetpointRate(int axis, float input) const
{
  if(axis == AXIS_YAW) input *= -1.f;
  return _rates.getSetpoint(axis, input);
}

void Controller::beginInnerLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  float pidScale[] = { 1.f, 1.f, 1.f };
  if(_model.config.mixer.type == FC_MIXER_GIMBAL)
  {
    pidScale[AXIS_YAW] = 0.2f; // ROBOT
    pidScale[AXIS_PITCH] = 20.f; // ROBOT
  }

  const PidConfig& pc = _model.config.pid[axis];
  Pid& pid = _model.state.innerPid[axis];
  pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[axis];
  pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[axis];
  pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[axis];
  pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[axis];
  pid.iLimitLow = -_model.config.iterm.limit * 0.01f;
  pid.iLimitHigh = _model.config.iterm.limit * 0.01f;
  pid.oLimitLow = -0.66f;
  pid.oLimitHigh = 0.66f;
  pid.rate = pidFilterRate;
  pid.dtermNotchFilter.begin(_model.config.dterm.notchFilter, pidFilterRate);
  if(_model.config.dterm.dynLpfFilter.cutoff > 0) {
    pid.dtermFilter.begin(FilterConfig((FilterType)_model.config.dterm.filter.type, _model.config.dterm.dynLpfFilter.cutoff), pidFilterRate);
  } else {
    pid.dtermFilter.begin(_model.config.dterm.filter, pidFilterRate);
  }
  pid.dtermFilter2.begin(_model.config.dterm.filter2, pidFilterRate);
  pid.ftermFilter.begin(_model.config.input.filterDerivative, pidFilterRate);
  pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, _model.config.iterm.relaxCutoff), pidFilterRate);
  if(axis == AXIS_YAW) {
    pid.itermRelax = (_model.config.iterm.relax == ITERM_RELAX_RPY || _model.config.iterm.relax == ITERM_RELAX_RPY_INC) ? _model.config.iterm.relax : ITERM_RELAX_OFF;
    pid.ptermFilter.begin(_model.config.yaw.filter, pidFilterRate);
  } else {
    pid.itermRelax = _model.config.iterm.relax;
  }
  pid.begin();
}

void Controller::beginOuterLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  PidConfig& pc = _model.config.pid[FC_PID_LEVEL];
  Pid& pid = _model.state.outerPid[axis];
  pid.Kp = (float)pc.P * LEVEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * LEVEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * LEVEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * LEVEL_FTERM_SCALE;
  pid.iLimitHigh = Utils::toRad(_model.config.level.rateLimit * 0.1f);
  pid.iLimitLow = -pid.iLimitHigh;
  pid.oLimitHigh = Utils::toRad(_model.config.level.rateLimit);
  pid.oLimitLow = -pid.oLimitHigh;
  pid.rate = pidFilterRate;
  pid.ptermFilter.begin(_model.config.level.ptermFilter, pidFilterRate);
  //pid.iLimit = 0.3f; // ROBOT
  //pid.oLimit = 1.f;  // ROBOT
  pid.begin();
}

void Controller::beginAltHold()
{
  float itermCenter = std::clamp((int)_model.config.altHold.itermCenter, 10, 60) * 0.01f;
  float itermRange = itermCenter * std::clamp((int)_model.config.altHold.itermRange, 10, 60) * 0.01f;
  PidConfig& pc = _model.config.pid[FC_PID_VEL];
  Pid& pid = _model.state.innerPid[AXIS_THRUST];
  pid.Kp = (float)pc.P * VEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * VEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * VEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * VEL_FTERM_SCALE;
  pid.iLimitLow  = -1.0f + 2.0f * (itermCenter - itermRange);
  pid.iLimitHigh = -1.0f + 2.0f * (itermCenter + itermRange);
  pid.iReset = pid.iLimitLow;
  pid.rate = _model.state.loopTimer.rate;
  pid.dtermFilter.begin(FilterConfig(FILTER_PT1, 10), _model.state.loopTimer.rate);
  pid.ftermDerivative = false;
  pid.begin();
}

}
