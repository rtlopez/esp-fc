#ifndef _ESPFC_TELEMETRY_H_
#define _ESPFC_TELEMETRY_H_

#include "Model.h"

namespace Espfc {

class Telemetry
{
  public:
    Telemetry(Model& model, Stream& stream): _model(model), _stream(stream) {}
    int begin()
    {

    }

    int update()
    {
      unsigned long now = millis();
      if(_model.config.telemetry && _model.config.telemetryInterval >= 10 && _model.state.telemetryTimestamp + _model.config.telemetryInterval < now)
      {
        (*this)
          //<< _model.state.timestamp
          //<< _model.state.gyroBiasValid

          //<< _model.state.angle[1]
          //<< _model.state.desiredRotation
          //<< _model.state.desiredAngle
          //<< _model.state.gyroPose
          //<< _model.state.accelPose
          //<< _model.state.accelPose2
          //<< _model.state.accelRaw
          //<< _model.state.gyroRaw
          //<< _model.state.magRaw
          //<< _model.state.gyro[1]
          //<< _model.state.gyro[2]
          //<< _model.state.rate[1]
          //<< _model.state.rate[2]
          //<< _model.state.accel
          //<< _model.state.mag
          //<< _model.state.magPose
          //<< _model.state.pose


          //<< _model.state.inputUs[0]
          //<< _model.state.inputUs[1]
          //<< _model.state.inputUs[2]
          //<< _model.state.inputUs[3]

          //<< _model.state.input[0]
          //<< _model.state.input[1]
          //<< _model.state.input[2]
          //<< _model.state.input[3]
          //<< _model.state.input[5]
          //<< _model.state.input[6]
          //<< _model.state.input[7]

          //<< _model.state.inputDelay / 1000000.0f

          //<< _model.state.flightMode
          << _model.state.innerPid[AXIS_PITH].pTerm
          << _model.state.innerPid[AXIS_PITH].iTerm
          << _model.state.innerPid[AXIS_PITH].dTerm
          //<< _model.state.innerPid[AXIS_YAW].pScale
          //<< _model.state.innerPid[AXIS_YAW].iScale

          << _model.state.outerPid[AXIS_PITH].pTerm
          << _model.state.outerPid[AXIS_PITH].iTerm
          << _model.state.outerPid[AXIS_PITH].dTerm

          //<< _model.state.balanceAngle[1]

          //<< _model.state.desiredAngle[1]
          //<< _model.state.desiredAngle[2]

          //<< _model.state.desiredRotation[1]
          //<< _model.state.desiredRotation[2]

          //<< _model.state.desiredRate[1]
          //<< _model.state.desiredRate[2]

          //<< _model.state.output[0]
          //<< _model.state.output[1]
          //<< _model.state.output[2]
          //<< _model.state.output[3]

          //<< _model.state.outputUs[0]
          //<< _model.state.outputUs[1]
          //<< _model.state.outputUs[2]
          //<< _model.state.outputUs[3]

          //<< _model.state.inputUs[4]
          //<< _model.state.inputUs[5]
          //<< _model.state.inputUs[6]
          //<< _model.state.inputUs[7]

          //<< _model.state.magCalibrationData[0][0]
          //<< _model.state.magCalibrationData[0][1]
          //<< _model.config.magCalibrationOffset[0]
          //<< _model.config.magCalibrationScale[0]

          //<< _model.state.magCalibrationData[1][0]
          //<< _model.state.magCalibrationData[1][1]
          //<< _model.config.magCalibrationOffset[1]
          //<< _model.config.magCalibrationScale[1]

          //<< _model.state.magCalibrationData[2][0]
          //<< _model.state.magCalibrationData[2][1]
          //<< _model.config.magCalibrationOffset[2]
          //<< _model.config.magCalibrationScale[2]

          //<< _model.config.magCalibration
        ;
        println();
        _model.state.telemetryTimestamp = now;
        return 1;
      }
      return 0;
    }

  private:
    template<typename T>
    Telemetry& operator<<(T v)
    {
      _stream.print(v); _stream.print(' ');
      return *this;
    }

    Telemetry& operator<<(long v)
    {
      _stream.print(v); _stream.print(' ');
      return *this;
    }

    Telemetry& operator<<(float v)
    {
      _stream.print(v, 4); _stream.print(' ');
      return *this;
    }

    Telemetry& operator<<(const VectorFloat& v)
    {
      (*this) << v.x << v.y << v.z;
      return *this;
    }

    Telemetry& operator<<(const VectorInt16& v)
    {
      (*this) << v.x << v.y << v.z;
      return *this;
    }

    Telemetry& operator<<(const Quaternion& q)
    {
      (*this) << q.w << q.x << q.y << q.z;
      return *this;
    }

    void println()
    {
      _stream.println();
    }

    Model& _model;
    Stream& _stream;
};

}

#endif
