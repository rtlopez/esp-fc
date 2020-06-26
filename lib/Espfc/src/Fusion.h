#ifndef _ESPFC_FUSION_H_
#define _ESPFC_FUSION_H_

#include "Model.h"
#include "Madgwick.h"
#include "Mahony.h"

namespace Espfc {

class Fusion
{
  public:
    Fusion(Model& model): _model(model), _first(true) {}
    int begin()
    {
      _model.state.gyroPoseQ = Quaternion();

      _madgwick.begin(_model.state.accelTimer.rate);
      _madgwick.setKp(_model.config.fusion.gain * 0.05f);

      _mahony.begin(_model.state.accelTimer.rate);
      _mahony.setKp(_model.config.fusion.gain * 0.05f);

      _model.logger.info().log(F("FUSION")).log(FPSTR(FusionConfig::getModeName((FusionMode)_model.config.fusion.mode))).logln(_model.config.fusion.gain);

      return 1;
    }

    void restoreGain()
    {
      _madgwick.setKp(_model.config.fusion.gain * 0.002f);
      _mahony.setKp(_model.config.fusion.gain * 0.002f);
    }

    int update()
    {
      Stats::Measure measure(_model.state.stats, COUNTER_IMU_FUSION);

      if(_model.accelActive())
      {
        switch(_model.config.fusion.mode)
        {
          case FUSION_MADGWICK:
            madgwickFusion();
            break;
          case FUSION_MAHONY:
            mahonyFusion();
            break;
          case FUSION_LERP:
            lerpFusion();
            break;
          case FUSION_RTQF:
            updatePoseFromAccelMag();
            rtqfFusion();
            break;
          case FUSION_KALMAN:
            //updatePoseFromAccelMag();
            kalmanFusion();
            break;
          case FUSION_COMPLEMENTARY:
            //updatePoseFromAccelMag();
            complementaryFusion();
            break;
          case FUSION_SIMPLE:
            simpleFusion();
            break;
          case FUSION_EXPERIMENTAL:
            experimentalFusion();
            break;
          case FUSION_NONE:
          default:
            ;
         }
       }
       //else madgwickFusion1();

       if(_model.config.debugMode == DEBUG_ALTITUDE)
       {
         _model.state.debug[0] = lrintf(degrees(_model.state.angle[0]) * 10);
         _model.state.debug[1] = lrintf(degrees(_model.state.angle[1]) * 10);
         _model.state.debug[2] = lrintf(degrees(_model.state.angle[2]) * 10);
       }
       return 1;
    }

    void updateDelayed()
    {
      Stats::Measure measure(_model.state.stats, COUNTER_IMU_FUSION2);
      switch(_model.config.fusion.mode)
      {
        case FUSION_MADGWICK:
          madgwickFusion2();
          break;
        case FUSION_MAHONY:
          mahonyFusion2();
          break;
      }
    }

    void experimentalFusion()
    {
      // Experiment: workaround for 90 deg limit on pitch[y] axis
      Quaternion r = Quaternion::lerp(Quaternion(), _model.state.accel.accelToQuaternion(), 0.5);
      _model.state.angle.eulerFromQuaternion(r);
      _model.state.angle *= 2.f;
    }

    void simpleFusion()
    {
      _model.state.pose = _model.state.accel.accelToEuler();
      _model.state.angle.x = _model.state.pose.x;
      _model.state.angle.y = _model.state.pose.y;
      _model.state.angle.z += _model.state.gyroTimer.intervalf * _model.state.gyro.z;
      if(_model.state.angle.z > PI) _model.state.angle.z -= TWO_PI;
      if(_model.state.angle.z < -PI) _model.state.angle.z += TWO_PI;
    }

    void kalmanFusion()
    {
      _model.state.pose = _model.state.accel.accelToEuler();
      _model.state.pose.z = _model.state.angle.z;
      const float dt = _model.state.gyroTimer.intervalf;
      for(size_t i = 0; i < 3; i++)
      {
        float angle = _model.state.kalman[i].getAngle(_model.state.pose.get(i), _model.state.gyro.get(i), dt);
        _model.state.angle.set(i, angle);
        //_model.state.rate.set(i, _model.state.kalman[i].getRate());
      }
      _model.state.angleQ = _model.state.angle.eulerToQuaternion();
    }

    void complementaryFusion()
    {
      _model.state.pose = _model.state.accel.accelToEuler();
      _model.state.pose.z = _model.state.angle.z;
      const float dt = _model.state.gyroTimer.intervalf;
      const float alpha = 0.002f;
      for(size_t i = 0; i < 3; i++)
      {
        float angle = (_model.state.angle[i] + _model.state.gyro[i] * dt) * (1.f - alpha) + _model.state.pose[i] * alpha;
        if(angle > PI) angle -= TWO_PI;
        if(angle < -PI) angle += TWO_PI;
        _model.state.angle.set(i, angle);
      }
      _model.state.angleQ = _model.state.angle.eulerToQuaternion();
      //_model.state.angle.eulerFromQuaternion(_model.state.angleQ); // causes NaN
    }

    void complementaryFusionOld()
    {
      const float alpha = 0.01f;
      const float dt = _model.state.gyroTimer.intervalf;
      _model.state.pose = _model.state.accel.accelToEuler();
      _model.state.pose.z = _model.state.angle.z;
      _model.state.angle = (_model.state.angle + _model.state.gyro * dt) * (1.f - alpha) + _model.state.pose * alpha;
      _model.state.angleQ = _model.state.angle.eulerToQuaternion();
    }

    void rtqfFusion()
    {
      float slerpPower = 0.001f;
      if(_first)
      {
        _model.state.angle = _model.state.pose;
        _model.state.angleQ = _model.state.poseQ;
        _first = false;
        return;
      }

      float timeDelta = _model.state.gyroTimer.intervalf;
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
    }

    void updatePoseFromAccelMag()
    {
      _model.state.pose = _model.state.accel.accelToEuler();
      //_model.state.accelPose = _model.state.pose;

      if(_model.magActive())
      {
        // Quaternion q = _model.state.pose.eulerToQuaternion();
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
    void lerpFusion()
    {
      float correctionAlpha = 0.001f; // 0 - 1 => gyro - accel

      _model.state.accelPose = _model.state.accel.accelToEuler();
      _model.state.accelPoseQ = _model.state.accelPose.eulerToQuaternion();

      if(_model.magActive())
      {
        // use yaw from mag
        VectorFloat mag = _model.state.mag.getRotated(_model.state.accelPoseQ);
        _model.state.accelPose.z = -atan2(mag.y, mag.x);
      }
      else
      {
        _model.state.accelPose.z = _model.state.gyroPose.z;
      }
      _model.state.accelPoseQ = _model.state.accelPose.eulerToQuaternion();

      //_model.state.accelPose.eulerFromQuaternion(_model.state.accelPoseQ);

      // predict new state
      Quaternion rotation = (_model.state.gyro * _model.state.gyroTimer.intervalf).eulerToQuaternion();
      _model.state.gyroPoseQ = (_model.state.gyroPoseQ * rotation).getNormalized();

      // drift compensation
      _model.state.gyroPoseQ = Quaternion::lerp(_model.state.gyroPoseQ, _model.state.accelPoseQ, correctionAlpha);

      // calculate euler vectors for accel and position
      _model.state.gyroPose.eulerFromQuaternion(_model.state.gyroPoseQ);
    }

    void madgwickFusion()
    {
      if(_model.magActive())
      {
        _madgwick.update(
          _model.state.gyroImu.x, _model.state.gyroImu.y, _model.state.gyroImu.z,
          _model.state.accel.x, _model.state.accel.y, _model.state.accel.z,
          _model.state.mag.x,   _model.state.mag.y,   _model.state.mag.z
        );
      }
      else
      {
        _madgwick.update(
          _model.state.gyroImu.x, _model.state.gyroImu.y, _model.state.gyroImu.z,
          _model.state.accel.x, _model.state.accel.y, _model.state.accel.z
        );
      }
      _model.state.angleQ = _madgwick.getQuaternion();
      _model.state.angle  = _madgwick.getEuler();
    }

    void madgwickFusion1()
    {
      // prediction phase
      _madgwick.update(
        _model.state.gyroImu.x, _model.state.gyroImu.y, _model.state.gyroImu.z,
        0.f, 0.f, 0.f
      );
      _model.state.angleQ = _madgwick.getQuaternion();
      _model.state.angle  = _madgwick.getEuler();
    }

    void madgwickFusion2()
    {
      //float mag = _model.state.accel.getMagnitude();
      //_model.state.debug[0] = lrintf(mag * 1000);
      //if(mag > 1.5f || mag < 0.75f) return; // skip at high overloads or weightlessness

      // correction phase
      _madgwick.update(
        0.f, 0.f, 0.f,
        _model.state.accel.x, _model.state.accel.y, _model.state.accel.z
      );
    }

    void mahonyFusion()
    {
      if(_model.magActive())
      {
        _mahony.update(
          _model.state.gyroImu.x, _model.state.gyroImu.y, _model.state.gyroImu.z,
          _model.state.accel.x, _model.state.accel.y, _model.state.accel.z,
          _model.state.mag.x,   _model.state.mag.y,   _model.state.mag.z
        );
      }
      else
      {
        _mahony.update(
          _model.state.gyroImu.x,  _model.state.gyroImu.y,  _model.state.gyroImu.z,
          _model.state.accel.x, _model.state.accel.y, _model.state.accel.z
        );
      }
      _model.state.angleQ = _mahony.getQuaternion();
      _model.state.angle  = _mahony.getEuler();
    }

    void mahonyFusion1()
    {
      // prediction phase
      _mahony.update(
        _model.state.gyroImu.x, _model.state.gyroImu.y, _model.state.gyroImu.z,
        0.f, 0.f, 0.f
      );
      _model.state.angleQ = _mahony.getQuaternion();
      _model.state.angle  = _mahony.getEuler();
    }

    void mahonyFusion2()
    {
      //float mag = _model.state.accel.getMagnitude();
      //_model.state.debug[0] = lrintf(mag * 1000);
      //if(mag > 1.5f || mag < 0.75f) return; // skip at high overloads or weightlessness

      // correction phase
      _mahony.update(
        0.f, 0.f, 0.f,
        _model.state.accel.x, _model.state.accel.y, _model.state.accel.z
      );
    }

  private:
    Model& _model;
    bool _first;
    Madgwick _madgwick;
    Mahony _mahony;
};

}

#endif
