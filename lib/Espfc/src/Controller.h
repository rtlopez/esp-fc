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
      if(!_model.state.newGyroData/* && !_model.state.newInputData*/) return 0;
      outerLoop();
      innerLoop();
    }

    void outerLoop()
    {
      if(_model.state.flightMode == MODE_ANGLE)
      {
        _model.state.desiredAngle = VectorFloat(
          _model.state.input[AXIS_ROLL] * _model.state.angleMax[AXIS_ROLL],
          _model.state.input[AXIS_PITH] * _model.state.angleMax[AXIS_PITH],
          _model.state.angle.z * 1
        );
        _model.state.desiredAngleQ = _model.state.desiredAngle.eulerToQuaternion();
        _model.state.desiredRotationQ = (_model.state.angleQ.getConjugate() * _model.state.desiredAngleQ).getNormalized();
        _model.state.desiredRotationQ = Quaternion::lerp(Quaternion(), _model.state.desiredRotationQ, 0.5f);
        _model.state.desiredRotation.eulerFromQuaternion(_model.state.desiredRotationQ);
      }
      else if(_model.state.flightMode == MODE_ANGLE_SIMPLE)
      {
        if(false && _model.config.modelFrame == FRAME_BALANCE_ROBOT && fabs(_model.state.input[AXIS_PITH]) < 0.05)
        {
          float balanceAngle = _model.state.balanceAngle.get(AXIS_PITH) + _model.state.gyroSampleIntervalFloat * _model.state.desiredRate[AXIS_PITH];
          balanceAngle = Math::bound(balanceAngle, _model.state.angleMax[AXIS_PITH] * -0.6f, _model.state.angleMax[AXIS_PITH] * 0.6f);
          _model.state.balanceAngle.set(1, balanceAngle);
        }
        _model.state.desiredAngle = VectorFloat(
          _model.state.input[AXIS_ROLL] * _model.state.angleMax[AXIS_ROLL],
          _model.state.input[AXIS_PITH] * _model.state.angleMax[AXIS_PITH],
          _model.state.angle.z * 1
        );
        _model.state.desiredAngleQ = _model.state.desiredAngle.eulerToQuaternion();
        _model.state.desiredRotation = (_model.state.desiredAngle + _model.state.balanceAngle - _model.state.angle) * 0.5f;
        _model.state.desiredRotationQ = _model.state.desiredRotation.eulerToQuaternion();
      }

      // only pitch and roll
      for(size_t i = 0; i < 2; ++i)
      {
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
          case MODE_RATE:
            _model.state.desiredRate[i] = _model.state.input[i];
            _model.state.outerPid[i].iTerm = 0;
            break;
          case MODE_ANGLE:
          case MODE_ANGLE_SIMPLE:
            {
              float angle = _model.state.angle[i] / _model.state.angleMax[i];
              float rotation = _model.state.desiredRotation[i] / _model.state.angleMax[i] * 2.f;
              _model.state.desiredRate[i] = _model.config.outerPid[i].update(rotation, angle, _model.state.gyroSampleIntervalFloat, _model.state.outerPid[i]);
            }
            break;
          default: // off
            _model.state.desiredRate[i] = 0.f;
            _model.state.outerPid[i].iTerm = 0;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
          _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW];
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
          _model.state.outerPid[AXIS_YAW].iTerm = 0;
          _model.state.outerPid[AXIS_THRUST].iTerm = 0;
          break;
        default:
          _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW];
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
      }
    }

    void innerLoop()
    {
      for(size_t i = 0; i < 2; ++i)
      {
        float rate = _model.state.rate[i] / _model.state.rateMax[i];
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
            _model.state.output[i] = _model.state.desiredRate[i];
            _model.state.innerPid[i].iTerm = 0;
            break;
          case MODE_ANGLE_SIMPLE:
          case MODE_ANGLE:
          case MODE_RATE:
            _model.state.output[i] = _model.config.innerPid[i].update(_model.state.desiredRate[i], rate, _model.state.gyroSampleIntervalFloat, _model.state.innerPid[i]);
            break;
          default: // off
            _model.state.output[i] = 0.f;
            _model.state.innerPid[i].iTerm = 0;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
          _model.state.output[AXIS_YAW] = _model.state.desiredRate[AXIS_YAW];
          _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
          break;
        case MODE_ANGLE_SIMPLE:
        case MODE_ANGLE:
        case MODE_RATE:
          {
            float rate = _model.state.rate[AXIS_YAW] / _model.state.rateMax[AXIS_YAW];
            _model.state.output[AXIS_YAW] = _model.config.innerPid[AXIS_YAW].update(_model.state.desiredRate[AXIS_YAW], rate, _model.state.gyroSampleIntervalFloat, _model.state.innerPid[AXIS_YAW]);
            _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
          }
          break;
        default: // off
          _model.state.output[AXIS_YAW] = 0.f;
          _model.state.output[AXIS_THRUST] = -1.f;
          _model.state.innerPid[AXIS_YAW].iTerm = 0;
          _model.state.innerPid[AXIS_THRUST].iTerm = 0;
      }
    }

  private:
    Model& _model;
};

}

#endif
