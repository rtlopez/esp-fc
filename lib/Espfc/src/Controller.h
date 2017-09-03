#ifndef _ESPFC_CONTROLLER_H_
#define _ESPFC_CONTROLLER_H_

#include "Model.h"

namespace Espfc {

class Controller
{
  public:
    Controller(Model& model): _model(model) {}
    int begin()
    {
      for(size_t i = 0; i < AXES; ++i)
      {
        _model.state.rateMax[i] = radians(_model.config.rateMax[i]);
        _model.state.angleMax[i] = radians(_model.config.angleMax[i]);
      }
    }

    int update()
    {
      if(!_model.state.gyroChanged) return 0;
      if(_model.state.gyroIteration % _model.config.pidSync != 0) return 0;

      uint32_t now = micros();
      _model.state.loopIteration++;
      _model.state.loopDt = (now - _model.state.loopTimestamp) / 1000000;
      _model.state.loopTimestamp = now;
      _model.state.loopChanged = true;

      resetIterm();
      outerLoop();
      innerLoop();
    }

    void outerLoop()
    {
      for(size_t i = 0; i < 2; ++i)
      {
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
            _model.state.desiredRate[i] = _model.state.input[i];
            break;
          case MODE_RATE:
            _model.state.desiredRate[i] = _model.state.input[i] * _model.state.rateMax[i];
            break;
          case MODE_ANGLE: // TODO
          case MODE_OFF:
          default:
            _model.state.desiredRate[i] = 0.f;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
          _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW];
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
          break;
        case MODE_RATE:
          _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW] * _model.state.rateMax[AXIS_YAW];
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
          break;
        case MODE_ANGLE: // TODO
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
            _model.state.output[i] = _model.config.innerPid[i].update(_model.state.desiredRate[i], _model.state.rate[i], _model.state.gyroDt, _model.state.innerPid[i]);
            break;
          case MODE_OFF:
          default: // off
            _model.state.output[i] = 0.f;
            _model.state.innerPid[i].iTerm = 0;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
        case MODE_RATE:
        case MODE_ANGLE:
          _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
          break;
        case MODE_OFF:
        default: // off
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

  private:
    Model& _model;
};

}

#endif
