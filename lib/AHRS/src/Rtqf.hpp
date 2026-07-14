#pragma once

#include "helper_3dmath.hpp"

// https://github.com/smukkejohan/RTIMULib/tree/master/RTIMULib
// do not use, experimental, not tested enough

class Rtqf
{
public:
  Rtqf();

  void begin(float sampleFrequency);
  void setKp(float kp);

  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  void update(float gx, float gy, float gz, float ax, float ay, float az);

  const Quaternion& getQuaternion() const;

private:
  void updatePoseFromAccelMag(float ax, float ay, float az, float mx, float my, float mz);
  void applyRtqf(float gx, float gy, float gz);

  Quaternion _quaternion{};
  Quaternion _poseQ{};
  float _dt;
  float _slerpPower;
  bool _first;
};
