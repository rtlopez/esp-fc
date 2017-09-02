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
      if(!_model.state.newGyroData) return 0;
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
            _model.state.output[i] = _model.config.innerPid[i].update(_model.state.desiredRate[i], _model.state.rate[i], _model.state.gyroSampleIntervalFloat, _model.state.innerPid[i]);
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

  private:
    Model& _model;
};

}

#endif
