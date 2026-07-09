#pragma once

#include "helper_3dmath.h"
#include <algorithm>
#include <cmath>

// https://github.com/smukkejohan/RTIMULib/tree/master/RTIMULib

class Rtqf
{
public:
  Rtqf(): _first(true), _dt(1.0f), _slerpPower(0.001f) {}

  void begin(float sampleFrequency)
  {
    _dt = 1.0f / sampleFrequency;
    _first = true;
  }

  void setKp(float kp)
  {
    _slerpPower = kp;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
  {
    updatePoseFromAccelMag(ax, ay, az, mx, my, mz);
    applyRtqf(gx, gy, gz);
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az)
  {
    update(gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f);
  }

  const Quaternion& getQuaternion() const
  {
    return _quaternion;
  }

private:
  void updatePoseFromAccelMag(float ax, float ay, float az, float mx, float my, float mz)
  {
    auto pose = VectorFloat{ax, ay, az}.accelToEuler();

    if (mx != 0.0f || my != 0.0f || mz != 0.0f)
    {
      // since Z is always 0, it can be optimized a bit
      // see VectorFloat::eulerToQuaternion() for reference
      float cosX2 = cos(pose.x / 2.0f);
      float sinX2 = sin(pose.x / 2.0f);
      float cosY2 = cos(pose.y / 2.0f);
      float sinY2 = sin(pose.y / 2.0f);
      Quaternion q{cosX2 * cosY2, sinX2 * cosY2, cosX2 * sinY2, -sinX2 * sinY2};
      VectorFloat m = VectorFloat{mx, my, mz}.getRotated(q);
      pose.z = -atan2(m.y, m.x);
    }
    else
    {
      pose.z = _euler.z;
    }
    _poseQ = Quaternion::ensureSign(pose.eulerToQuaternion(), _poseQ);
    _pose.eulerFromQuaternion(_poseQ);
  }

  void applyRtqf(float gx, float gy, float gz)
  {
    if (_first)
    {
      _quaternion = _poseQ;
      _euler.eulerFromQuaternion(_quaternion);
      _first = false;
      return;
    }

    Quaternion q = _quaternion;
    VectorFloat gyro = VectorFloat{gx, gy, gz};

    float qw = q.w;
    float qx = q.x;
    float qy = q.y;
    float qz = q.z;

    float gx2 = gyro.x / 2.0;
    float gy2 = gyro.y / 2.0;
    float gz2 = gyro.z / 2.0;

    // Predict new state
    q.w = qw + (-gx2 * qx - gy2 * qy - gz2 * qz) * _dt;
    q.x = qx + (gx2 * qw + gz2 * qy - gy2 * qz) * _dt;
    q.y = qy + (gy2 * qw - gz2 * qx + gx2 * qz) * _dt;
    q.z = qz + (gz2 * qw + gy2 * qx - gx2 * qy) * _dt;

    // calculate rotation delta
    Quaternion rotationError = q.getConjugate() * _poseQ;
    rotationError.normalize();

    // take it to the power (0 to 1) to give the desired amount of correction
    float theta = acos(std::clamp(rotationError.w, -1.0f, 1.0f));
    float sinPowerTheta = sin(theta * _slerpPower);
    float cosPowerTheta = cos(theta * _slerpPower);

    VectorFloat rotationVector(rotationError.x, rotationError.y, rotationError.z);
    rotationVector.normalize();

    Quaternion rotationPower;
    rotationPower.w = cosPowerTheta;
    rotationPower.x = sinPowerTheta * rotationVector.x;
    rotationPower.y = sinPowerTheta * rotationVector.y;
    rotationPower.z = sinPowerTheta * rotationVector.z;
    rotationPower.normalize();

    //  multiple this by predicted value to get result
    q = q * rotationPower;
    q.normalize();

    _quaternion = Quaternion::ensureSign(q, _quaternion);
    _euler.eulerFromQuaternion(_quaternion);
  }

  bool _first;
  float _dt;
  float _slerpPower;
  Quaternion _quaternion{};
  VectorFloat _euler{};
  Quaternion _poseQ{};
  VectorFloat _pose{};
};
