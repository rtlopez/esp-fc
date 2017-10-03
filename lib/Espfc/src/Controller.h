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
        _model.state.innerPid[i].dtermFilter.begin(_model.config.dtermFilterType, _model.config.dtermFilterCutFreq, _model.state.loopSampleRate);
        _model.state.outerPid[i].dtermFilter.begin(_model.config.dtermFilterType, _model.config.dtermFilterCutFreq, _model.state.loopSampleRate);
      }
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
      switch(_model.state.flightMode)
      {
        case MODE_ANGLE:
        case MODE_ANGLE_SIMPLE:
          _model.state.desiredAngle = VectorFloat(
            _model.state.input[AXIS_ROLL] * _model.config.angleMax[AXIS_ROLL],
            _model.state.input[AXIS_PITCH] * _model.config.angleMax[AXIS_PITCH],
            _model.state.angle[AXIS_YAW]
          );
          _model.state.controlAngle = _model.state.angle;
          break;
        default:
        ;
      }

      for(size_t i = 0; i < 2; ++i)
      {
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
            _model.state.desiredRate[i] = _model.state.input[i];
            break;
          case MODE_RATE:
            _model.state.desiredRate[i] = calculateSetpointRate(i, _model.state.input[i]);
            break;
          case MODE_ANGLE:
          case MODE_ANGLE_SIMPLE:
            _model.state.desiredRate[i] = _model.config.outerPid[i].update(_model.state.desiredAngle[i], _model.state.controlAngle[i], _model.state.loopDt, _model.state.outerPid[i]);
            break;
          case MODE_OFF:
          default:
            _model.state.desiredRate[i] = 0.f;
            _model.state.outerPid[i].iTerm = 0.f;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
          _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW];
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
          break;
        case MODE_RATE:
        case MODE_ANGLE:
        case MODE_ANGLE_SIMPLE:
          _model.state.desiredRate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, _model.state.input[AXIS_YAW]);
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
          break;
        case MODE_OFF:
        default:
          _model.state.desiredRate[AXIS_YAW] = 0.f;
          _model.state.desiredRate[AXIS_THRUST] = -1.f;
      }
    }

    void innerLoop()
    {
      for(size_t i = 0; i < 3; ++i)
      {
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
            _model.state.output[i] = _model.state.desiredRate[i];
            _model.state.innerPid[i].iTerm = 0;
            break;
          case MODE_RATE:
          case MODE_ANGLE:
          case MODE_ANGLE_SIMPLE:
            _model.state.output[i] = _model.config.innerPid[i].update(_model.state.desiredRate[i], _model.state.rate[i], _model.state.loopDt, _model.state.innerPid[i]);
            break;
          case MODE_OFF:
          default:
            _model.state.output[i] = 0.f;
            _model.state.innerPid[i].iTerm = 0;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
        case MODE_RATE:
        case MODE_ANGLE:
        case MODE_ANGLE_SIMPLE:
          _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
          break;
        case MODE_OFF:
        default:
          _model.state.output[AXIS_THRUST] = -1.f;
      }
    }

    void resetIterm()
    {
      if(!_model.config.lowThrottleZeroIterm) return;
      if(_model.state.input[AXIS_THRUST] < _model.config.lowThrottleTreshold)
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
