#ifndef _ESPFC_FUSION_H_
#define _ESPFC_FUSION_H_

#include "Model.h"

namespace Espfc {

class Fusion
{
  public:
    Fusion(Model& model): _model(model), _first(true) {}
    int begin() {}
    int update()
    {
       //if(!_model.state.gyroBiasValid) return 0;
       updatePoseFromAccelMag(_model.state.accel, _model.state.mag);
       _model.state.gyroPose += _model.state.gyro * _model.state.gyroSampleIntervalFloat;
       kalmanFusion();
       //slerpFusion();
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
    }

    void complementaryFusion()
    {
      //TODO: implement complementary filter
    }

    void slerpFusion()
    {
      float slerpPower = 0.02;
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
      fusionQPose.x = qx + (x2 * qs + z2 * qy - y2 * qz) * timeDelta;
      fusionQPose.y = qy + (y2 * qs - z2 * qx + x2 * qz) * timeDelta;
      fusionQPose.z = qz + (z2 * qs + y2 * qx - x2 * qy) * timeDelta;

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
    }

    void updatePoseFromAccelMag(const VectorFloat& accel, const VectorFloat& mag)
    {
      VectorFloat r = _model.state.accelPose = accel.accelToEuler();

      // Quaternion q = r.eulerToQuaternion();
      // since Z is always 0, it can be optimized a bit
      float cosX2 = cos(r.x / 2.0f);
      float sinX2 = sin(r.x / 2.0f);
      float cosY2 = cos(r.y / 2.0f);
      float sinY2 = sin(r.y / 2.0f);

      Quaternion q;
      q.w = cosX2 * cosY2;
      q.x = sinX2 * cosY2;
      q.y = cosX2 * sinY2;
      q.z = -sinX2 * sinY2;

      //VectorFloat m = mag.getRotated(q);
      Quaternion m(0, mag.x, mag.y, mag.z);
      m = q * m * q.getConjugate();

      r.z = -atan2(m.y, m.x);

      _model.state.pose = r;
      _model.state.poseQ = r.eulerToQuaternion();
    }

  private:
    Model& _model;
    bool _first;
};

}

#endif
