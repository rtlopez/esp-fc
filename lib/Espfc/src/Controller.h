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
      unsigned long now = millis();

      if(_model.state.controllerTimestamp + _model.state.gyroSampleInterval > now) return 0;

      outerLoop();
      innerLoop();

      _model.state.controllerTimestamp = now;
    }

    void outerLoop()
    {
      // only pitch and roll
      for(size_t i = 0; i < 2; ++i)
      {
        float angle = _model.state.angle[i] / _model.state.angleMax[i];
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
          case MODE_RATE:
            _model.state.rateDesired[i] = _model.state.input[i];
            break;
          case MODE_ANGLE:
            _model.state.rateDesired[i] = _model.config.outerPid[i].update(_model.state.input[i], angle, _model.state.gyroSampleIntervalFloat, _model.state.outerPid[i]);
            break;
          default: // disarmed
            _model.state.rateDesired[i] = 0.f;
        }
      }
      _model.state.rateDesired[AXIS_YAW] = _model.state.input[AXIS_YAW];
      _model.state.rateDesired[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
    }

    void innerLoop()
    {
      for(size_t i = 0; i < 2; ++i)
      {
        float rate = _model.state.rate[i] / _model.state.rateMax[i];
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
            _model.state.output[i] = _model.state.rateDesired[i];
          case MODE_ANGLE:
          case MODE_RATE:
            _model.state.output[i] = _model.config.innerPid[i].update(_model.state.rateDesired[i], rate, _model.state.gyroSampleIntervalFloat, _model.state.innerPid[i]);
            break;
          default: // disarmed
            _model.state.output[i] = 0.f;
        }
      }

      if(_model.state.flightMode != MODE_DISARMED)
      {
        float rate = _model.state.rate[AXIS_YAW] / _model.state.rateMax[AXIS_YAW];
        _model.state.output[AXIS_YAW] = _model.config.innerPid[AXIS_YAW].update(_model.state.rateDesired[AXIS_YAW], rate, _model.state.gyroSampleIntervalFloat, _model.state.innerPid[AXIS_YAW]);

        // manual thrust
        _model.state.output[AXIS_THRUST] = _model.state.rateDesired[AXIS_THRUST];
      }
      else
      {
        _model.state.output[AXIS_YAW] = 0.f;
        _model.state.output[AXIS_THRUST] = -1.f;
      }
    }

  private:
    Model& _model;
};

}

#endif
