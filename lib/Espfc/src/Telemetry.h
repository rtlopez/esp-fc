#ifndef _ESPFC_TELEMETRY_H_
#define _ESPFC_TELEMETRY_H_

#include "Model.h"
#include "Hardware.h"

namespace Espfc {

class Telemetry
{
  public:
    Telemetry(Model& model): _model(model) {}

    int begin()
    {
      _stream = (Stream*)Hardware::getSerialPort(_model.config.telemetryPort);
      return 1;
    }

    int update()
    {
      if(!_stream || !_model.config.telemetry) return 0;
      //if(_model.state.gyroChanged) return 0;

      unsigned long now = millis();
      if(_model.config.telemetryInterval >= 10 && _model.state.telemetryTimestamp + _model.config.telemetryInterval < now)
      {
        (*this)
          //<< _model.state.timestamp
          //<< _model.state.armed
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
          //<< _model.state.gyro
          //<< _model.state.rate
          //<< _model.state.accel
          //<< _model.state.mag
          //<< _model.state.magPose
          //<< _model.state.pose

          //<< _model.state.inputUs[0]
          //<< _model.state.inputUs[1]
          //<< _model.state.inputUs[2]
          //<< _model.state.inputUs[3]
          //<< _model.state.inputUs[4]
          //<< _model.state.inputUs[5]
          //<< _model.state.inputUs[6]
          //<< _model.state.inputUs[7]

          //<< _model.state.input[0]
          //<< _model.state.input[1]
          //<< _model.state.input[2]
          //<< _model.state.input[3]
          //<< _model.state.input[4]
          //<< _model.state.input[5]
          //<< _model.state.input[6]
          //<< _model.state.input[7]

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

          //<< _model.state.velocity[1]
          //<< (_model.state.output[1] - _model.state.gyro[1] * 0.05f) * -1.f
          //<< (_model.state.output[1] + _model.state.gyro[1] * 0.05f) * -1.f

          //<< _model.state.outerPid[AXIS_PITCH].pTerm
          //<< _model.state.outerPid[AXIS_PITCH].iTerm
          //<< _model.state.outerPid[AXIS_PITCH].dTerm

          //<< _model.state.innerPid[AXIS_PITCH].pTerm
          //<< _model.state.innerPid[AXIS_PITCH].iTerm
          //<< _model.state.innerPid[AXIS_PITCH].dTerm

          //<< _model.state.gyro[1] * _model.state.gyroThrustScale
          //<< _model.state.gyroThrustScale

          //<< _model.state.innerPid[AXIS_YAW].pTerm
          //<< _model.state.innerPid[AXIS_YAW].iTerm
          //<< _model.state.innerPid[AXIS_YAW].dTerm

          //<< _model.state.outputUs[0]
          //<< _model.state.outputUs[1]
          //<< _model.state.outputUs[2]
          //<< _model.state.outputUs[3]

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
      if(!_stream) return *this;
      (*_stream).print(v);
      (*_stream).print(' ');
      return *this;
    }

    Telemetry& operator<<(long v)
    {
      if(!_stream) return *this;
      (*_stream).print(v);
      (*_stream).print(' ');
      return *this;
    }

    Telemetry& operator<<(float v)
    {
      if(!_stream) return *this;
      (*_stream).print(v, 4);
      (*_stream).print(' ');
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
      if(!_stream) return;
      (*_stream).println();
    }

    Model& _model;
    Stream * _stream;
};

}

#endif
