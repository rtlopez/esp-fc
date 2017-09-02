#ifndef _ESPFC_CONTROLLER_OLD_H_
#define _ESPFC_CONTROLLER_OLD_H_

#include "Model.h"

namespace Espfc {

class ControllerOld
{
  public:
    ControllerOld(Model& model): _model(model) {}
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
      navigationLoop();
      outerLoop();
      innerLoop();
    }

    void navigationLoop()
    {
      switch(_model.state.flightMode)
      {
        case MODE_DIRECT:
        case MODE_BALANCING_ANGLE:
        case MODE_BALANCING_ROBOT:
          {
            // estimate velocity from output
            VectorFloat velocity = VectorFloat(
              (_model.state.output[AXIS_ROLL]  - _model.state.gyro[AXIS_ROLL] * _model.state.gyroThrustScale)  * _model.config.velocityMax[AXIS_ROLL]  * -1.f,
              (_model.state.output[AXIS_PITCH] - _model.state.gyro[AXIS_PITCH] * _model.state.gyroThrustScale) * _model.config.velocityMax[AXIS_PITCH] * -1.f,
              0
            );
            _model.state.velocity = _model.state.velocity * (1.f - _model.config.velocityFilterAlpha) + velocity * _model.config.velocityFilterAlpha;
            _model.state.desiredVelocity = VectorFloat(
              _model.state.input[AXIS_ROLL]  * _model.config.velocityMax[AXIS_ROLL],
              _model.state.input[AXIS_PITCH] * _model.config.velocityMax[AXIS_PITCH],
              0
            );
          }
          break;
      }

      switch(_model.state.flightMode)
      {
        case MODE_ANGLE:
          _model.state.desiredAngle = VectorFloat(
            _model.state.input[AXIS_ROLL] * _model.state.angleMax[AXIS_ROLL],
            _model.state.input[AXIS_PITCH] * _model.state.angleMax[AXIS_PITCH],
            _model.state.angle.z * 1
          );
          _model.state.desiredAngleQ = _model.state.desiredAngle.eulerToQuaternion();
          _model.state.desiredRotationQ = (_model.state.angleQ.getConjugate() * _model.state.desiredAngleQ).getNormalized();
          _model.state.desiredRotationQ = Quaternion::lerp(Quaternion(), _model.state.desiredRotationQ, 0.5f);
          _model.state.desiredRotation.eulerFromQuaternion(_model.state.desiredRotationQ);
          break;
        case MODE_BALANCING_ANGLE:
        case MODE_ANGLE_SIMPLE:
          _model.state.desiredAngle = VectorFloat(
            _model.state.input[AXIS_ROLL] * _model.state.angleMax[AXIS_ROLL],
            _model.state.input[AXIS_PITCH] * _model.state.angleMax[AXIS_PITCH],
            _model.state.angle.z * 1
          );
          //_model.state.desiredAngleQ = _model.state.desiredAngle.eulerToQuaternion();
          //_model.state.desiredRotation = (_model.state.desiredAngle - _model.state.angle) * 0.5f;
          //_model.state.desiredRotationQ = _model.state.desiredRotation.eulerToQuaternion();
          break;
        default:
          break;
      }
    }

    void outerLoop()
    {
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
            {
              _model.state.desiredRate[i] = _model.config.outerPid[i].update(_model.state.desiredRotation[i] * 2.f, 0, _model.state.gyroSampleIntervalFloat, _model.state.outerPid[i]);
            }
            break;
          case MODE_ANGLE_SIMPLE:
            {
              float angle = _model.state.angle[i] / _model.state.angleMax[i];
              float angleDesired = _model.state.desiredAngle[i] / _model.state.angleMax[i];
              _model.state.desiredRate[i] = _model.config.outerPid[i].update(angleDesired, angle, _model.state.gyroSampleIntervalFloat, _model.state.outerPid[i]);
            }
            break;
          case MODE_BALANCING_ANGLE:
            // do noting
            break;
          case MODE_BALANCING_ROBOT:
            {
              float velocity = _model.state.velocity[i] / _model.config.velocityMax[i];
              float velocityDesired = _model.state.desiredVelocity[i] / _model.config.velocityMax[i];
              float angleDesired = _model.config.outerPid[i].update(velocityDesired, velocity, _model.state.gyroSampleIntervalFloat, _model.state.outerPid[i]);
              _model.state.desiredAngle.set(i, angleDesired);
            }
            break;
          default: // off
            _model.state.desiredRate[i] = 0.f;
            _model.state.outerPid[i].iTerm = 0;
        }
      }

      switch(_model.state.flightMode)
      {
        case MODE_OFF:
        case MODE_DIRECT:
          _model.state.outerPid[AXIS_YAW].iTerm = 0;
          _model.state.outerPid[AXIS_THRUST].iTerm = 0;
        default:
          _model.state.desiredRate[AXIS_YAW] = _model.state.input[AXIS_YAW];
          _model.state.desiredRate[AXIS_THRUST] = _model.state.input[AXIS_THRUST];
      }
    }

    void innerLoop()
    {
      for(size_t i = 0; i < 2; ++i)
      {
        switch(_model.state.flightMode)
        {
          case MODE_DIRECT:
            _model.state.output[i] = _model.state.desiredRate[i];
            _model.state.innerPid[i].iTerm = 0;
            break;
          case MODE_ANGLE_SIMPLE:
          case MODE_ANGLE:
          case MODE_RATE:
            {
              float rate = _model.state.rate[i] / _model.state.rateMax[i];
              _model.state.output[i] = _model.config.innerPid[i].update(_model.state.desiredRate[i], rate, _model.state.gyroSampleIntervalFloat, _model.state.innerPid[i]);
            }
            break;
          case MODE_BALANCING_ANGLE:
          case MODE_BALANCING_ROBOT:
            {
              float angle = _model.state.angle[i] / _model.state.angleMax[i];
              _model.state.desiredRate[i] = _model.state.output[i] = _model.config.innerPid[i].update(_model.state.desiredAngle[i], angle, _model.state.gyroSampleIntervalFloat, _model.state.innerPid[i]);
            }
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
          _model.state.innerPid[AXIS_YAW].iTerm = 0;
          _model.state.output[AXIS_THRUST] = _model.state.desiredRate[AXIS_THRUST];
          _model.state.innerPid[AXIS_THRUST].iTerm = 0;
          break;
        case MODE_BALANCING_ANGLE:
        case MODE_BALANCING_ROBOT:
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
          _model.state.innerPid[AXIS_YAW].iTerm = 0;
          _model.state.output[AXIS_THRUST] = -1.f;
          _model.state.innerPid[AXIS_THRUST].iTerm = 0;
      }
    }

  private:
    Model& _model;
};

}

#endif
