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
      _model.state.loopSampleRate = _model.state.gyroSampleRate / _model.config.loopSync;
      _model.state.loopSampleInterval = 1000000 / _model.state.loopSampleRate;
      for(size_t i = 0; i < 3; ++i)
      {
        _model.state.innerPid[i].configureFilter((FilterType)_model.config.dtermFilterType, _model.config.dtermFilterCutFreq, _model.state.loopSampleRate);
        _model.state.outerPid[i].configureFilter((FilterType)_model.config.dtermFilterType, _model.config.dtermFilterCutFreq, _model.state.loopSampleRate);
      }
      return 1;
    }

    int update()
    {
      _model.state.stats.start(COUNTER_OUTER_PID);
      resetIterm();
      outerLoop();
      _model.state.stats.end(COUNTER_OUTER_PID);

      _model.state.stats.start(COUNTER_INNER_PID);
      innerLoop();
      _model.state.stats.end(COUNTER_INNER_PID);

      return 1;
    }

    void outerLoop()
    {
      if(_model.isMode(MODE_ANGLE))
      {
        _model.state.desiredAngle = VectorFloat(
          _model.state.input[AXIS_ROLL] * radians(_model.config.angleLimit),
          _model.state.input[AXIS_PITCH] * radians(_model.config.angleLimit),
          _model.state.angle[AXIS_YAW]
        );
        _model.state.controlAngle = _model.state.angle;
        _model.state.desiredRate[AXIS_ROLL] = _model.state.outerPid[AXIS_ROLL].update(_model.state.desiredAngle[AXIS_ROLL], _model.state.controlAngle[AXIS_ROLL], _model.state.loopDt);
        _model.state.desiredRate[AXIS_PITCH] = _model.state.outerPid[AXIS_PITCH].update(_model.state.desiredAngle[AXIS_PITCH], _model.state.controlAngle[AXIS_PITCH], _model.state.loopDt);
      }
      else
      {
        _model.state.desiredRate[AXIS_ROLL] = calculateSetpointRate(AXIS_ROLL, _model.state.input[AXIS_ROLL]);
        _model.state.desiredRate[AXIS_PITCH] = calculateSetpointRate(AXIS_PITCH, _model.state.input[AXIS_PITCH]);
      }
      _model.state.desiredRate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input[AXIS_YAW]);
      _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
    }

    void innerLoop()
    {
      for(size_t i = 0; i <= AXIS_YAW; ++i)
      {
        _model.state.output[i] = _model.state.innerPid[i].update(_model.state.desiredRate[i], _model.state.rate[i], _model.state.loopDt);
      }
      _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
    }

    void resetIterm()
    {
      if(!_model.isMode(MODE_ARMED) || (_model.config.lowThrottleZeroIterm && _model.state.inputUs[AXIS_THRUST] < _model.config.lowThrottleTreshold))
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
      float rcRate = _model.config.inputRate[axis] / 100.0f;
      uint8_t rcExpo = _model.config.inputExpo[axis];

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
      if (_model.config.inputSuperRate[axis])
      {
        const float rcSuperfactor = 1.0f / (Math::bound(1.0f - (inputAbs * (_model.config.inputSuperRate[axis] / 100.0f)), 0.01f, 1.00f));
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
};

}

#endif
