#ifndef _ESPFC_FUSION_H_
#define _ESPFC_FUSION_H_

#include "Model.h"

namespace Espfc {

class Fusion
{
  public:
    Fusion(Model& model): _model(model), _first(true), _gyro_first(true) {}
    int begin() {}
    int update()
    {
      if(!_model.state.newGyroData) return 0;
      updateGyroPose();
      updatePoseFromAccelMag();
      switch(_model.config.fusionMode)
      {
        case FUSION_RTQF:
          rtqfFusion();
          break;
        case FUSION_KALMAN:
          kalmanFusion();
          break;
        case FUSION_COMPLEMENTARY:
        default:
          complementaryFusion();
          break;
       }
       return 1;
    }

    void kalmanFusion()
    {
      for(size_t i = 0; i < 3; i++)
      {
        float angle = _model.state.kalman[i].getAngle(_model.state.pose.get(i), _model.state.gyro.get(i), _model.state.gyroSampleIntervalFloat);
        _model.state.angle.set(i, angle);
        _model.state.rate.set(i, _model.state.kalman[i].getRate());
      }
      _model.state.angleQ = _model.state.angle.eulerToQuaternion();
    }

    void complementaryFusion()
    {
      float alpha = 0.02f;
      for(size_t i = 0; i < 3; i++)
      {
        if(i == 2) alpha = 0.f;
        float angle = (_model.state.angle.get(i) + _model.state.gyro.get(i) * _model.state.gyroSampleIntervalFloat) * (1.f - alpha);
        //if(angle > 0 && _model.state.pose.get(i) < 0) angle -= TWO_PI;
        //if(angle < 0 && _model.state.pose.get(i) > 0) angle += TWO_PI;
        angle += _model.state.pose.get(i) * alpha;
        _model.state.angle.set(i, angle);
      }
      _model.state.rate  = _model.state.gyro;
      _model.state.angleQ = _model.state.angle.eulerToQuaternion();
    }

    void complementaryFusionOld()
    {
      float alpha = 0.02f;
      _model.state.angle = (_model.state.angle + _model.state.gyro * _model.state.gyroSampleIntervalFloat) * (1.f - alpha) + _model.state.pose * alpha;
      _model.state.rate  = _model.state.gyro;
      _model.state.angleQ = _model.state.angle.eulerToQuaternion();
    }

    void rtqfFusion()
    {
      float slerpPower = 0.015f;
      if(_first)
      {
        _model.state.angle = _model.state.pose;
        _model.state.angleQ = _model.state.poseQ;
        _first = false;
        return;
      }

      float timeDelta = _model.state.gyroSampleIntervalFloat;
      Quaternion measuredQPose = _model.state.poseQ;
      Quaternion fusionQPose = _model.state.angleQ;
      VectorFloat fusionGyro = _model.state.gyro;

      float qs, qx, qy, qz;
      qs = fusionQPose.w;
      qx = fusionQPose.x;
      qy = fusionQPose.y;
      qz = fusionQPose.z;

      float x2, y2, z2;
      x2 = fusionGyro.x / 2.0;
      y2 = fusionGyro.y / 2.0;
      z2 = fusionGyro.z / 2.0;

      // Predict new state
      fusionQPose.w = qs + (-x2 * qx - y2 * qy - z2 * qz) * timeDelta;
      fusionQPose.x = qx + ( x2 * qs + z2 * qy - y2 * qz) * timeDelta;
      fusionQPose.y = qy + ( y2 * qs - z2 * qx + x2 * qz) * timeDelta;
      fusionQPose.z = qz + ( z2 * qs + y2 * qx - x2 * qy) * timeDelta;

      // calculate rotation delta
      Quaternion rotationDelta = fusionQPose.getConjugate() * measuredQPose;
      rotationDelta.normalize();

      // take it to the power (0 to 1) to give the desired amount of correction
      float theta = acos(rotationDelta.w);
      float sinPowerTheta = sin(theta * slerpPower);
      float cosPowerTheta = cos(theta * slerpPower);

      VectorFloat rotationUnitVector(rotationDelta.x, rotationDelta.y, rotationDelta.z);
      rotationUnitVector.normalize();

      Quaternion rotationPower;
      rotationPower.w = cosPowerTheta;
      rotationPower.x = sinPowerTheta * rotationUnitVector.x;
      rotationPower.y = sinPowerTheta * rotationUnitVector.y;
      rotationPower.z = sinPowerTheta * rotationUnitVector.z;
      rotationPower.normalize();

      //  multiple this by predicted value to get result
      fusionQPose = fusionQPose * rotationPower;
      fusionQPose.normalize();

      _model.state.angleQ = fusionQPose;
      _model.state.angle.eulerFromQuaternion(fusionQPose);
      _model.state.rate  = _model.state.gyro;
    }

    void updatePoseFromAccelMag()
    {
      _model.state.pose = _model.state.accel.accelToEuler();
      _model.state.accelPose = _model.state.pose;

      if(_model.config.magEnable)
      {
        // Quaternion q = r.eulerToQuaternion();
        // since Z is always 0, it can be optimized a bit
        float cosX2 = cos(_model.state.pose.x / 2.0f);
        float sinX2 = sin(_model.state.pose.x / 2.0f);
        float cosY2 = cos(_model.state.pose.y / 2.0f);
        float sinY2 = sin(_model.state.pose.y / 2.0f);

        Quaternion q;
        q.w =  cosX2 * cosY2;
        q.x =  sinX2 * cosY2;
        q.y =  cosX2 * sinY2;
        q.z = -sinX2 * sinY2;

        VectorFloat m = _model.state.mag.getRotated(q);
        _model.state.magPose = m;
        _model.state.pose.z = -atan2(m.y, m.x);
      }
      else
      {
        _model.state.pose.z = _model.state.angle.z;
      }

      //_model.state.accelPose.z = _model.state.gyroPose.z;
      _model.state.accelPoseQ = _model.state.accelPose.eulerToQuaternion();
      _model.state.accelPose.eulerFromQuaternion(_model.state.accelPoseQ);

      _model.state.poseQ = _model.state.pose.eulerToQuaternion();
      _model.state.pose.eulerFromQuaternion(_model.state.poseQ);

      //return;
      //  check for quaternion aliasing. If the quaternion has the wrong sign
      //  the kalman filter will be very unhappy.
      int maxIndex = -1;
      float maxVal = -1000;

      for(int i = 0; i < 4; i++) {
        if(fabs(_model.state.poseQ.get(i)) > maxVal) {
          maxVal = fabs(_model.state.poseQ.get(i));
          maxIndex = i;
        }
      }

      //  if the biggest component has a different sign in the measured and kalman poses,
      //  change the sign of the measured pose to match.
      if(((_model.state.poseQ.get(maxIndex) < 0) && (_model.state.angleQ.get(maxIndex) > 0)) ||
         ((_model.state.poseQ.get(maxIndex) > 0) && (_model.state.angleQ.get(maxIndex) < 0))) {
        _model.state.poseQ.w = -_model.state.poseQ.w;
        _model.state.poseQ.x = -_model.state.poseQ.x;
        _model.state.poseQ.y = -_model.state.poseQ.y;
        _model.state.poseQ.z = -_model.state.poseQ.z;
        _model.state.pose.eulerFromQuaternion(_model.state.poseQ);
      }
    }

    // experimental
    void updateGyroPose()
    {
      /*
      _model.state.gyroPose += _model.state.gyro * _model.state.gyroSampleIntervalFloat;
      if(_model.state.gyroPose.x >  PI) _model.state.gyroPose.x -= TWO_PI;
      if(_model.state.gyroPose.x < -PI) _model.state.gyroPose.x += TWO_PI;
      if(_model.state.gyroPose.y >  PI) _model.state.gyroPose.y -= TWO_PI;
      if(_model.state.gyroPose.y < -PI) _model.state.gyroPose.y += TWO_PI;
      if(_model.state.gyroPose.z >  PI) _model.state.gyroPose.z -= TWO_PI;
      if(_model.state.gyroPose.z < -PI) _model.state.gyroPose.z += TWO_PI;
      */
      //_model.state.gyroPoseQ = _model.state.gyroPose.eulerToQuaternion();

      if(_gyro_first)
      {
        _model.state.gyroPoseQ = _model.state.gyroPose.eulerToQuaternion();
        _gyro_first = false;
      }
      Quaternion rotation = (_model.state.gyro * _model.state.gyroSampleIntervalFloat).eulerToQuaternion();
      _model.state.gyroPoseQ = _model.state.gyroPoseQ * rotation;

      Quaternion delta = _model.state.gyroPoseQ.getConjugate() * _model.state.poseQ;
      Quaternion zero;
      Quaternion apply = lerp(zero, delta, 0.01);

      _model.state.gyroPoseQ = _model.state.gyroPoseQ * apply;

      _model.state.gyroPose.eulerFromQuaternion(_model.state.gyroPoseQ);
    }

    Quaternion lerp(const Quaternion &q1, const Quaternion &q2, float t)
    {
      return (q1 * (1-t) + q2 * t).getNormalized();
    }

  private:
    Model& _model;
    bool _first;
    bool _gyro_first;
};

}

#endif
