#pragma once

#include "helper_3dmath.h"
#include <algorithm>
#include <cmath>

// https://github.com/smukkejohan/RTIMULib/tree/master/RTIMULib
// do not use, experimental, not tested enough

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
      const auto q = Quaternion{cosX2 * cosY2, sinX2 * cosY2, cosX2 * sinY2, -sinX2 * sinY2};
      const auto m = VectorFloat{mx, my, mz}.getRotated(q);
      pose.z = -atan2(m.y, m.x);
    }
    else
    {
      const auto& q = _quaternion;
      pose.z = atan2f(q.x * q.y + q.w * q.z, 0.5f - q.y * q.y - q.z * q.z);
    }
    _poseQ = Quaternion::ensureSign(pose.eulerToQuaternion(), _poseQ);
  }

  void applyRtqf(float gx, float gy, float gz)
  {
    if (_first)
    {
      _quaternion = _poseQ;
      _first = false;
      return;
    }

    auto q = _quaternion;
    auto g = VectorFloat{gx, gy, gz} * 0.5f;

    float qw = q.w;
    float qx = q.x;
    float qy = q.y;
    float qz = q.z;

    // Predict new state
    q.w = qw - (g.x * qx + g.y * qy + g.z * qz) * _dt;
    q.x = qx + (g.x * qw + g.z * qy - g.y * qz) * _dt;
    q.y = qy + (g.y * qw - g.z * qx + g.x * qz) * _dt;
    q.z = qz + (g.z * qw + g.y * qx - g.x * qy) * _dt;

    // calculate rotation delta
    auto error = q.getConjugate() * _poseQ;

    // skip rotation if the error is too small, to avoid numerical issues
    auto mag = error.x * error.x + error.y * error.y + error.z * error.z;
    if (mag > 1e-9f)
    {
      // Prefer the shorter rotation path to avoid flips near 180° ambiguity.
      if (error.w < 0.0f)
      {
        error = {-error.w, -error.x, -error.y, -error.z};
      }

      error.normalize();

      // take it to the power (0 to 1) to give the desired amount of correction
      float theta = acos(std::clamp(error.w, -1.0f, 1.0f));
      float sinPowerTheta = sin(theta * _slerpPower);
      float cosPowerTheta = cos(theta * _slerpPower);

      const auto rotationVector = VectorFloat{error.x, error.y, error.z}.getNormalized();

      // clang-format off
      const auto rotationPower = Quaternion{
        cosPowerTheta,
        rotationVector.x * sinPowerTheta,
        rotationVector.y * sinPowerTheta,
        rotationVector.z * sinPowerTheta
      }.getNormalized();
      // clang-format on

      //  multiple this by predicted value to get result
      q = q * rotationPower;
    }

    _quaternion = Quaternion::ensureSign(q, _quaternion).getNormalized();
  }

  bool _first;
  float _dt;
  float _slerpPower;
  Quaternion _quaternion{};
  Quaternion _poseQ{};
};
