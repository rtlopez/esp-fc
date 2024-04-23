#include "Controller.h"
#include "Math/Utils.h"

namespace Espfc {

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
  if(_model.config.debugMode == DEBUG_PIDLOOP)
  {
    startTime = micros();
    _model.state.debug[0] = startTime - _model.state.loopTimer.last;
  }

  {
    Stats::Measure(_model.state.stats, COUNTER_OUTER_PID);
    resetIterm();
    if(_model.config.mixerType == FC_MIXER_GIMBAL)
    {
      outerLoopRobot();
    }
    else
    {
      outerLoop();
    }
  }

  {
    Stats::Measure(_model.state.stats, COUNTER_INNER_PID);
    if(_model.config.mixerType == FC_MIXER_GIMBAL)
    {
      innerLoopRobot();
    }
    else
    {
      innerLoop();
    }
  }

  if(_model.config.debugMode == DEBUG_PIDLOOP)
  {
    _model.state.debug[2] = micros() - startTime;
  }

  return 1;
}

void Controller::outerLoopRobot()
{
  const float speedScale = 2.f;
  const float gyroScale = 0.1f;
  const float speed = _speedFilter.update(_model.state.output[AXIS_PITCH] * speedScale + _model.state.gyro[AXIS_PITCH] * gyroScale);
  float angle = 0;

  if(true || _model.isActive(MODE_ANGLE))
  {
    angle = _model.state.input[AXIS_PITCH] * radians(_model.config.angleLimit);
  }
  else
  {
    angle = _model.state.outerPid[AXIS_PITCH].update(_model.state.input[AXIS_PITCH], speed) * radians(_model.config.angleRateLimit);
  }
  _model.state.desiredAngle.set(AXIS_PITCH, angle);
  _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW] * radians(_model.config.angleRateLimit);

  if(_model.config.debugMode == DEBUG_ANGLERATE)
  {
    _model.state.debug[0] = speed * 1000;
    _model.state.debug[1] = lrintf(degrees(angle) * 10);
  }
}

void Controller::innerLoopRobot()
{
  //VectorFloat v(0.f, 0.f, 1.f);
  //v.rotate(_model.state.angleQ);
  //const float angle = acos(v.z);
  const float angle = std::max(abs(_model.state.angle[AXIS_PITCH]), abs(_model.state.angle[AXIS_ROLL]));

  const bool stabilize = angle < radians(_model.config.angleLimit);
  if(stabilize)
  {
    _model.state.output[AXIS_PITCH] = _model.state.innerPid[AXIS_PITCH].update(_model.state.desiredAngle[AXIS_PITCH], _model.state.angle[AXIS_PITCH]);
    _model.state.output[AXIS_YAW]   = _model.state.innerPid[AXIS_YAW].update(_model.state.desiredRate[AXIS_YAW], _model.state.gyro[AXIS_YAW]);
  }
  else
  {
    resetIterm();
    _model.state.output[AXIS_PITCH] = 0.f;
    _model.state.output[AXIS_YAW] = 0.f;
  }

  if(_model.config.debugMode == DEBUG_ANGLERATE)
  {
    _model.state.debug[2] = lrintf(degrees(_model.state.angle[AXIS_PITCH]) * 10);
    _model.state.debug[3] = lrintf(_model.state.output[AXIS_PITCH] * 1000);
  }
}

void FAST_CODE_ATTR Controller::outerLoop()
{
  if(_model.isActive(MODE_ANGLE))
  {
    _model.state.desiredAngle = VectorFloat(
      _model.state.input[AXIS_ROLL] * radians(_model.config.angleLimit),
      _model.state.input[AXIS_PITCH] * radians(_model.config.angleLimit),
      _model.state.angle[AXIS_YAW]
    );
    _model.state.desiredRate[AXIS_ROLL]  = _model.state.outerPid[AXIS_ROLL].update(_model.state.desiredAngle[AXIS_ROLL], _model.state.angle[AXIS_ROLL]);
    _model.state.desiredRate[AXIS_PITCH] = _model.state.outerPid[AXIS_PITCH].update(_model.state.desiredAngle[AXIS_PITCH], _model.state.angle[AXIS_PITCH]);
    // disable fterm in angle mode
    _model.state.innerPid[AXIS_ROLL].fScale = 0.f;
    _model.state.innerPid[AXIS_PITCH].fScale = 0.f;
  }
  else
  {
    _model.state.desiredRate[AXIS_ROLL] = calculateSetpointRate(AXIS_ROLL, _model.state.input[AXIS_ROLL]);
    _model.state.desiredRate[AXIS_PITCH] = calculateSetpointRate(AXIS_PITCH, _model.state.input[AXIS_PITCH]);
  }
  _model.state.desiredRate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input[AXIS_YAW]);
  _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];

  if(_model.config.debugMode == DEBUG_ANGLERATE)
  {
    for(size_t i = 0; i < 3; ++i)
    {
      _model.state.debug[i] = lrintf(degrees(_model.state.desiredRate[i]));
    }
  }
}

void FAST_CODE_ATTR Controller::innerLoop()
{
  const float tpaFactor = getTpaFactor();
  for(size_t i = 0; i <= AXIS_YAW; ++i)
  {
    _model.state.output[i] = _model.state.innerPid[i].update(_model.state.desiredRate[i], _model.state.gyro[i]) * tpaFactor;
    //_model.state.debug[i] = lrintf(_model.state.innerPid[i].fTerm * 1000);
  }
  _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];

  if(_model.config.debugMode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Math::toDeg(_model.state.innerPid[0].itermRelaxBase));
    _model.state.debug[1] = lrintf(_model.state.innerPid[0].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Math::toDeg(_model.state.innerPid[0].iTermError));
    _model.state.debug[3] = lrintf(_model.state.innerPid[0].iTerm * 1000.0f);
  }
}

float Controller::getTpaFactor() const
{
  if(_model.config.tpaScale == 0) return 1.f;
  float t = Math::clamp(_model.state.inputUs[AXIS_THRUST], (float)_model.config.tpaBreakpoint, 2000.f);
  return Math::map(t, (float)_model.config.tpaBreakpoint, 2000.f, 1.f, 1.f - ((float)_model.config.tpaScale * 0.01f));
}

void Controller::resetIterm()
{
  if(!_model.isActive(MODE_ARMED)   // when not armed
    || (!_model.isAirModeActive() && _model.config.lowThrottleZeroIterm && _model.isThrottleLow()) // on low throttle (not in air mode)
  )
  {
    for(size_t i = 0; i < AXES; i++)
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
