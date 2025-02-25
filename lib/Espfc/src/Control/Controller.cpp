#include "Control/Controller.h"
#include "Utils/Math.hpp"
#include <algorithm>

namespace Espfc::Control {

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
    _model.state.output.ch[AXIS_THRUST] = _model.state.setpoint.rate[AXIS_THRUST];
  }

  if(_model.config.debug.mode == DEBUG_STACK)
  {
    _model.state.debug[0] = std::clamp(lrintf(_model.state.setpoint.rate[AXIS_THRUST] * 1000.0f), -3000l, 30000l);  // gps trust
    _model.state.debug[1] = std::clamp(lrintf(_model.state.altitude.height * 100.0f), -3000l, 30000l);              // baro alt
    _model.state.debug[2] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].error * 1000.0f), -3000l, 30000l); // gps alt
    _model.state.debug[3] = std::clamp(lrintf(_model.state.altitude.vario * 100.0f), -3000l, 30000l);               // vario
    _model.state.debug[4] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].pTerm * 1000.0f), -3000l, 30000l); // not used 1
    _model.state.debug[5] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].iTerm * 1000.0f), -3000l, 30000l); // not used 2
    _model.state.debug[6] = std::clamp(lrintf(_model.state.innerPid[AXIS_THRUST].dTerm * 1000.0f), -3000l, 30000l); // not used 3
    _model.state.debug[7] = std::clamp(lrintf(_model.state.baro.pressureRaw - 100000.0f), -30000l, 30000l);         // not used 4
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

  return thrust * 0.5f; // climb/descend rate factor 0.5 m/s
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
    _model.state.innerPid[AXIS_THRUST].resetIterm();
  }
}

float Controller::calculateSetpointRate(int axis, float input) const
{
  if(axis == AXIS_YAW) input *= -1.f;
  return _rates.getSetpoint(axis, input);
}

}
