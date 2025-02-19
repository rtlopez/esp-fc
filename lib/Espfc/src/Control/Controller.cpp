#include "Control/Controller.h"
#include "Utils/Math.hpp"

namespace Espfc {

namespace Control {

Controller::Controller(Model& model): _model(model) {}

int Controller::begin()
{
  _rates.begin(_model.config.input);
  _speedFilter.begin(FilterConfig(FILTER_BIQUAD, 10), _model.state.loopTimer.rate);
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
    if(_model.config.mixer.type == FC_MIXER_GIMBAL)
    {
      outerLoopRobot();
    }
    else
    {
      outerLoop();
    }
  }

  {
    Utils::Stats::Measure(_model.state.stats, COUNTER_INNER_PID);
    if(_model.config.mixer.type == FC_MIXER_GIMBAL)
    {
      innerLoopRobot();
    }
    else
    {
      innerLoop();
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
  if(_model.isModeActive(MODE_ANGLE))
  {
    _model.state.setpoint.angle = VectorFloat(
      _model.state.input.ch[AXIS_ROLL] * Utils::toRad(_model.config.level.angleLimit),
      _model.state.input.ch[AXIS_PITCH] * Utils::toRad(_model.config.level.angleLimit),
      _model.state.attitude.euler[AXIS_YAW]
    );
    _model.state.setpoint.rate[AXIS_ROLL]  = _model.state.outerPid[AXIS_ROLL].update(_model.state.setpoint.angle[AXIS_ROLL], _model.state.attitude.euler[AXIS_ROLL]);
    _model.state.setpoint.rate[AXIS_PITCH] = _model.state.outerPid[AXIS_PITCH].update(_model.state.setpoint.angle[AXIS_PITCH], _model.state.attitude.euler[AXIS_PITCH]);
    // disable fterm in angle mode
    _model.state.innerPid[AXIS_ROLL].fScale = 0.f;
    _model.state.innerPid[AXIS_PITCH].fScale = 0.f;
  }
  else
  {
    _model.state.setpoint.rate[AXIS_ROLL] = calculateSetpointRate(AXIS_ROLL, _model.state.input.ch[AXIS_ROLL]);
    _model.state.setpoint.rate[AXIS_PITCH] = calculateSetpointRate(AXIS_PITCH, _model.state.input.ch[AXIS_PITCH]);
  }
  _model.state.setpoint.rate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input.ch[AXIS_YAW]);
  _model.state.setpoint.rate[AXIS_THRUST] = _model.state.input.ch[AXIS_THRUST];

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
  const float tpaFactor = getTpaFactor();
  for(size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    _model.state.output.ch[i] = _model.state.innerPid[i].update(_model.state.setpoint.rate[i], _model.state.gyro.adc[i]) * tpaFactor;
    //_model.state.debug[i] = lrintf(_model.state.innerPid[i].fTerm * 1000);
  }
  _model.state.output.ch[AXIS_THRUST] = _model.state.setpoint.rate[AXIS_THRUST];

  if(_model.config.debug.mode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(_model.state.innerPid[0].itermRelaxBase));
    _model.state.debug[1] = lrintf(_model.state.innerPid[0].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Utils::toDeg(_model.state.innerPid[0].iTermError));
    _model.state.debug[3] = lrintf(_model.state.innerPid[0].iTerm * 1000.0f);
  }
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
    for(size_t i = 0; i < AXIS_COUNT_RPYT; i++)
    {
      _model.state.innerPid[i].iTerm = 0;
      _model.state.outerPid[i].iTerm = 0;
    }
  }
}

float Controller::calculateSetpointRate(int axis, float input)
{
  if(axis == AXIS_YAW) input *= -1.f;
  return _rates.getSetpoint(axis, input);
}

}

}
