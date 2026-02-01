#include "Control/Fusion.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Control {

Fusion::Fusion(Model& model): _model(model), _first(true) {}

int Fusion::begin()
{
  _model.state.gyroPoseQ = Quaternion();

  _madgwick.begin(_model.state.accel.timer.rate);
  _madgwick.setKp(_model.config.fusion.gain * 0.05f);

  _mahony.begin(_model.state.accel.timer.rate);
  _mahony.setKp(_model.config.fusion.gain * 0.1f);
  _mahony.setKi(_model.config.fusion.gainI * 0.1f);

  _model.logger.info().log(F("FUSION")).log(FPSTR(FusionConfig::getModeName((FusionMode)_model.config.fusion.mode))).logln(_model.config.fusion.gain);

  return 1;
}

void Fusion::restoreGain()
{
  _madgwick.setKp(_model.config.fusion.gain * 0.005f);
  _mahony.setKp(_model.config.fusion.gain * 0.02f);
  _mahony.setKi(_model.config.fusion.gainI * 0.02f);
}

int FAST_CODE_ATTR Fusion::update()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_IMU_FUSION);

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
        break;
      }

      //_model.state.accel.world = _model.state.accel.adc.getRotated(_model.state.attitude.quaternion);
      _model.state.accel.world = VectorFloat(0.f, 0.f, 1.f).getRotated(_model.state.attitude.quaternion);

      const Quaternion& q = _model.state.attitude.quaternion;
      _model.state.attitude.cosTheta = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    }

    if(_model.config.debug.mode == DEBUG_ACCELEROMETER)
    {
      _model.state.debug[3] = lrintf(_model.state.accel.world[0] * 2048);
      _model.state.debug[4] = lrintf(_model.state.accel.world[1] * 2048);
      _model.state.debug[5] = lrintf(_model.state.accel.world[2] * 2048);
      _model.state.debug[6] = lrintf(_model.state.attitude.cosTheta * 1000.f);
    }

    if(_model.config.debug.mode == DEBUG_AC_CORRECTION)
    {
      _model.state.debug[0] = lrintf(Utils::toDeg(_model.state.attitude.euler[0]) * 10);
      _model.state.debug[1] = lrintf(Utils::toDeg(_model.state.attitude.euler[1]) * 10);
      _model.state.debug[2] = lrintf(Utils::toDeg(_model.state.attitude.euler[2]) * 10);
    }
    return 1;
}

void Fusion::experimentalFusion()
{
  // Experiment: workaround for 90 deg limit on pitch[y] axis
  Quaternion r = Quaternion::lerp(Quaternion(), _model.state.accel.adc.accelToQuaternion(), 0.5);
  _model.state.attitude.euler.eulerFromQuaternion(r);
  _model.state.attitude.euler *= 2.f;
}

void Fusion::simpleFusion()
{
  _model.state.pose = _model.state.accel.adc.accelToEuler();
  _model.state.attitude.euler.x = _model.state.pose.x;
  _model.state.attitude.euler.y = _model.state.pose.y;
  _model.state.attitude.euler.z += _model.state.gyro.timer.intervalf * _model.state.gyro.adc.z;
  if(_model.state.attitude.euler.z > PI) _model.state.attitude.euler.z -= TWO_PI;
  if(_model.state.attitude.euler.z < -PI) _model.state.attitude.euler.z += TWO_PI;
}

void Fusion::kalmanFusion()
{
  _model.state.pose = _model.state.accel.adc.accelToEuler();
  _model.state.pose.z = _model.state.attitude.euler.z;
  const float dt = _model.state.gyro.timer.intervalf;
  for(size_t i = 0; i < 3; i++)
  {
    float angle = _model.state.kalman[i].getAngle(_model.state.pose.get(i), _model.state.gyro.adc.get(i), dt);
    _model.state.attitude.euler.set(i, angle);
    //_model.state.rate.set(i, _model.state.kalman[i].getRate());
  }
  _model.state.attitude.quaternion = _model.state.attitude.euler.eulerToQuaternion();
}

void Fusion::complementaryFusion()
{
  _model.state.pose = _model.state.accel.adc.accelToEuler();
  _model.state.pose.z = _model.state.attitude.euler.z;
  const float dt = _model.state.gyro.timer.intervalf;
  const float alpha = 0.002f;
  for(size_t i = 0; i < 3; i++)
  {
    float angle = (_model.state.attitude.euler[i] + _model.state.gyro.adc[i] * dt) * (1.f - alpha) + _model.state.pose[i] * alpha;
    if(angle > PI) angle -= TWO_PI;
    if(angle < -PI) angle += TWO_PI;
    _model.state.attitude.euler.set(i, angle);
  }
  _model.state.attitude.quaternion = _model.state.attitude.euler.eulerToQuaternion();
  //_model.state.attitude.euler.eulerFromQuaternion(_model.state.attitude.quaternion); // causes NaN
}

void Fusion::complementaryFusionOld()
{
  const float alpha = 0.01f;
  const float dt = _model.state.gyro.timer.intervalf;
  _model.state.pose = _model.state.accel.adc.accelToEuler();
  _model.state.pose.z = _model.state.attitude.euler.z;
  _model.state.attitude.euler = (_model.state.attitude.euler + _model.state.gyro.adc * dt) * (1.f - alpha) + _model.state.pose * alpha;
  _model.state.attitude.quaternion = _model.state.attitude.euler.eulerToQuaternion();
}

void Fusion::rtqfFusion()
{
  float slerpPower = 0.001f;
  if(_first)
  {
    _model.state.attitude.euler = _model.state.pose;
    _model.state.attitude.quaternion = _model.state.poseQ;
    _first = false;
    return;
  }

  float timeDelta = _model.state.gyro.timer.intervalf;
  Quaternion measuredQPose = _model.state.poseQ;
  Quaternion fusionQPose = _model.state.attitude.quaternion;
  VectorFloat fusionGyro = _model.state.gyro.adc;

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

  _model.state.attitude.quaternion = fusionQPose;
  _model.state.attitude.euler.eulerFromQuaternion(fusionQPose);
}

void Fusion::updatePoseFromAccelMag()
{
  _model.state.pose = _model.state.accel.adc.accelToEuler();
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

    VectorFloat m = _model.state.mag.adc.getRotated(q);
    _model.state.mag.pose = m;
    _model.state.pose.z = -atan2(m.y, m.x);
  }
  else
  {
    _model.state.pose.z = _model.state.attitude.euler.z;
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
  if(((_model.state.poseQ.get(maxIndex) < 0) && (_model.state.attitude.quaternion.get(maxIndex) > 0)) ||
      ((_model.state.poseQ.get(maxIndex) > 0) && (_model.state.attitude.quaternion.get(maxIndex) < 0))) {
    _model.state.poseQ.w = -_model.state.poseQ.w;
    _model.state.poseQ.x = -_model.state.poseQ.x;
    _model.state.poseQ.y = -_model.state.poseQ.y;
    _model.state.poseQ.z = -_model.state.poseQ.z;
    _model.state.pose.eulerFromQuaternion(_model.state.poseQ);
  }
}

// experimental
void Fusion::lerpFusion()
{
  float correctionAlpha = 0.001f; // 0 - 1 => gyro - accel

  _model.state.accelPose = _model.state.accel.adc.accelToEuler();
  _model.state.accelPoseQ = _model.state.accelPose.eulerToQuaternion();

  if(_model.magActive())
  {
    // use yaw from mag
    VectorFloat mag = _model.state.mag.adc.getRotated(_model.state.accelPoseQ);
    _model.state.accelPose.z = -atan2(mag.y, mag.x);
  }
  else
  {
    _model.state.accelPose.z = _model.state.gyroPose.z;
  }
  _model.state.accelPoseQ = _model.state.accelPose.eulerToQuaternion();

  //_model.state.accelPose.eulerFromQuaternion(_model.state.accelPoseQ);

  // predict new state
  Quaternion rotation = (_model.state.gyro.adc * _model.state.gyro.timer.intervalf).eulerToQuaternion();
  _model.state.gyroPoseQ = (_model.state.gyroPoseQ * rotation).getNormalized();

  // drift compensation
  _model.state.gyroPoseQ = Quaternion::lerp(_model.state.gyroPoseQ, _model.state.accelPoseQ, correctionAlpha);

  // calculate euler vectors for accel and position
  _model.state.gyroPose.eulerFromQuaternion(_model.state.gyroPoseQ);
}

void Fusion::madgwickFusion()
{
  if(false && _model.magActive())
  {
    _madgwick.update(
      _model.state.attitude.rate.x, _model.state.attitude.rate.y, _model.state.attitude.rate.z,
      _model.state.accel.adc.x, _model.state.accel.adc.y, _model.state.accel.adc.z,
      _model.state.mag.adc.x,   _model.state.mag.adc.y,   _model.state.mag.adc.z
    );
  }
  else
  {
    _madgwick.update(
      _model.state.attitude.rate.x, _model.state.attitude.rate.y, _model.state.attitude.rate.z,
      _model.state.accel.adc.x, _model.state.accel.adc.y, _model.state.accel.adc.z
    );
  }
  _model.state.attitude.quaternion = _madgwick.getQuaternion();
  _model.state.attitude.euler  = _madgwick.getEuler();
}

void FAST_CODE_ATTR Fusion::mahonyFusion()
{
  if(false && _model.magActive())
  {
    _mahony.update(
      _model.state.attitude.rate.x, _model.state.attitude.rate.y, _model.state.attitude.rate.z,
      _model.state.accel.adc.x, _model.state.accel.adc.y, _model.state.accel.adc.z,
      _model.state.mag.adc.x,   _model.state.mag.adc.y,   _model.state.mag.adc.z
    );
  }
  else
  {
    _mahony.update(
      _model.state.attitude.rate.x,  _model.state.attitude.rate.y,  _model.state.attitude.rate.z,
      _model.state.accel.adc.x, _model.state.accel.adc.y, _model.state.accel.adc.z
    );
  }
  _model.state.attitude.quaternion = _mahony.getQuaternion();
  _model.state.attitude.euler  = _mahony.getEuler();
}

}

}
