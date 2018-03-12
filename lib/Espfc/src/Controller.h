#ifndef _ESPFC_CONTROLLER_H_
#define _ESPFC_CONTROLLER_H_

#include "Model.h"

#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

namespace Espfc {

class Controller
{
  public:
    Controller(Model& model): _model(model) {}

    int begin()
    {
      FilterConfig fc;
      fc.freq = 10;
      fc.type = FILTER_BIQUAD;
      _speedFilter.begin(fc, _model.state.loopTimer.rate);
      return 1;
    }

    int update()
    {
      _model.state.stats.start(COUNTER_OUTER_PID);
      resetIterm();
      if(_model.config.mixerType == MIXER_GIMBAL)
      {
        outerLoopRobot();
      }
      else
      {
        outerLoop();
      }
      _model.state.stats.end(COUNTER_OUTER_PID);

      _model.state.stats.start(COUNTER_INNER_PID);
      if(_model.config.mixerType == MIXER_GIMBAL)
      {
        innerLoopRobot();
      }
      else
      {
        innerLoop();
      }
      _model.state.stats.end(COUNTER_INNER_PID);

      return 1;
    }

    void outerLoopRobot()
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
        angle = _model.state.outerPid[AXIS_PITCH].update(_model.state.input[AXIS_PITCH], speed, _model.state.loopTimer.getDelta()) * radians(_model.config.angleRateLimit);
      }
      _model.state.desiredAngle.set(AXIS_PITCH, angle);
      _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW] * radians(_model.config.angleRateLimit);

      if(_model.config.debugMode == DEBUG_ANGLERATE)
      {
        _model.state.debug[0] = speed * 1000;
        _model.state.debug[1] = lrintf(degrees(angle) * 10);
      }
    }

    void innerLoopRobot()
    {
      //VectorFloat v(0.f, 0.f, 1.f);
      //v.rotate(_model.state.angleQ);
      //const float angle = acos(v.z);
      const float angle = std::max(abs(_model.state.angle[AXIS_PITCH]), abs(_model.state.angle[AXIS_ROLL]));

      const bool stabilize = angle < radians(_model.config.angleLimit);
      if(stabilize)
      {
        const float dt = _model.state.loopTimer.getDelta();
        _model.state.output[AXIS_PITCH] = _model.state.innerPid[AXIS_PITCH].update(_model.state.desiredAngle[AXIS_PITCH], _model.state.angle[AXIS_PITCH], dt);
        _model.state.output[AXIS_YAW]   = _model.state.innerPid[AXIS_YAW].update(_model.state.desiredRate[AXIS_YAW], _model.state.rate[AXIS_YAW], dt);
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

    void outerLoop()
    {
      if(_model.isActive(MODE_ANGLE))
      {
        const float dt = _model.state.loopTimer.getDelta();
        _model.state.desiredAngle = VectorFloat(
          _model.state.input[AXIS_ROLL] * radians(_model.config.angleLimit),
          _model.state.input[AXIS_PITCH] * radians(_model.config.angleLimit),
          _model.state.angle[AXIS_YAW]
        );
        _model.state.desiredRate[AXIS_ROLL]  = _model.state.outerPid[AXIS_ROLL].update(_model.state.desiredAngle[AXIS_ROLL], _model.state.angle[AXIS_ROLL], dt);
        _model.state.desiredRate[AXIS_PITCH] = _model.state.outerPid[AXIS_PITCH].update(_model.state.desiredAngle[AXIS_PITCH], _model.state.angle[AXIS_PITCH], dt);
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

    void innerLoop()
    {
      const float dt = _model.state.loopTimer.getDelta();
      const float tpaFactor = getTpaFactor();
      for(size_t i = 0; i <= AXIS_YAW; ++i)
      {
        _model.state.output[i] = _model.state.innerPid[i].update(_model.state.desiredRate[i], _model.state.rate[i], dt) * tpaFactor;
      }
      _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
    }

    float getTpaFactor() const
    {
      if(_model.config.tpaScale == 0) return 1.f;
      float t = Math::bound(_model.state.inputUs[AXIS_THRUST], (float)_model.config.tpaBreakpoint, 2000.f);
      return Math::map(t, (float)_model.config.tpaBreakpoint, 2000.f, 1.f, 1.f - ((float)_model.config.tpaScale * 0.01f));
    }

    void resetIterm()
    {
      if(!_model.isActive(MODE_ARMED)   // when disarmed
        || (!_model.isActive(MODE_AIRMODE) && _model.config.lowThrottleZeroIterm && _model.isThrottleLow()) // on low throttle (not in air mode)
      )
      {
        for(size_t i = 0; i < AXES; i++)
        {
          _model.state.innerPid[i].iTerm = 0;
          _model.state.outerPid[i].iTerm = 0;
        }
      }
    }

    float calculateSetpointRate(int axis, float input)
    {
      if(axis == AXIS_YAW) input *= -1.f;

      float rcRate = _model.config.input.rate[axis] / 100.0f;
      const uint8_t rcExpo = _model.config.input.expo[axis];

      if (rcRate > 2.0f)
      {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
      }

      const float inputAbs = std::abs(input);
      if(rcExpo)
      {
        const float expof = rcExpo / 100.0f;
        input = input * power3(inputAbs) * expof + input * (1.f - expof);
      }

      float angleRate = 200.0f * rcRate * input;
      if (_model.config.input.superRate[axis])
      {
        const float rcSuperfactor = 1.0f / (Math::bound(1.0f - (inputAbs * (_model.config.input.superRate[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
      }
      return radians(Math::bound(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT)); // Rate limit protection (deg/sec)
    }

  private:
    float power3(float x)
    {
      return x * x * x;
    }

    Model& _model;
    Filter _speedFilter;
};

}

#endif
