#ifndef _ESPFC_MODEL_STATE_H_
#define _ESPFC_MODEL_STATE_H_

#include <Arduino.h>

#include "Stats.h"
#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "Filter.h"
#include "Stats.h"
#include "Timer.h"

namespace Espfc {

class BuzzerState
{
  public:
    BuzzerState(): idx(0) {}

    void play(BuzzerEvent e) // play continously, repeat while condition is true
    {
      if(!empty()) return;
      push(e);
    }

    void push(BuzzerEvent e) // play once
    {
      if(full()) return;
      if(beeperMask & (1 << (e - 1)))
      {
        events[idx++] = e;
      }
    }

    BuzzerEvent pop()
    {
      if(empty()) return BEEPER_SILENCE;
      return events[--idx];
    }

    bool empty() const
    {
      return idx == 0;
    }

    bool full() const
    {
      return idx >= BUZZER_MAX_EVENTS;
    }

    Timer timer;
    BuzzerEvent events[BUZZER_MAX_EVENTS];
    size_t idx;
    int32_t beeperMask;
};

class BatteryState
{
  public:
    bool warn(int vbatCellWarning) const
    {
      if(voltage < 20) return false; // no battery connected
      return !samples && cellVoltage < vbatCellWarning;
    }

    int16_t rawVoltage;
    uint8_t voltage;
    uint8_t cellVoltage;
    int8_t cells;
    int8_t samples;
    Timer timer;
};

// working data
struct ModelState
{
  VectorInt16 gyroRaw;
  VectorInt16 accelRaw;
  VectorInt16 magRaw;

  VectorFloat gyro;
  VectorFloat accel;
  VectorFloat mag;

  VectorFloat gyroPose;
  Quaternion gyroPoseQ;
  VectorFloat accelPose;
  VectorFloat accelPose2;
  Quaternion accelPoseQ;
  VectorFloat magPose;

  VectorFloat pose;
  Quaternion poseQ;

  VectorFloat rate;
  VectorFloat angle;
  Quaternion angleQ;

  Filter gyroFilter[3];
  Filter gyroNotch1Filter[3];
  Filter gyroNotch2Filter[3];

  Filter accelFilter[3];
  Filter magFilter[3];

  VectorFloat velocity;
  VectorFloat desiredVelocity;

  VectorFloat desiredAngle;
  Quaternion desiredAngleQ;

  float desiredRate[AXES];

  Pid innerPid[AXES];
  Pid outerPid[AXES];

  bool inputLinkValid;
  float inputUs[INPUT_CHANNELS];
  float input[INPUT_CHANNELS];

  float output[OUTPUT_CHANNELS];
  int16_t outputUs[OUTPUT_CHANNELS];

  // other state
  Kalman kalman[AXES];
  VectorFloat accelPrev;

  float accelScale;
  VectorFloat accelBias;
  float accelBiasAlpha;
  int accelBiasSamples;

  float gyroScale;
  VectorFloat gyroBias;
  float gyroBiasAlpha;
  long gyroBiasSamples;

  int8_t gyroDivider;

  Timer gyroTimer;
  bool gyroUpdate;

  Timer loopTimer;
  bool loopUpdate;

  Timer mixerTimer;
  bool mixerUpdate;
  float minThrottle;
  float maxThrottle;
  bool digitalOutput;

  Timer actuatorTimer;

  Timer magTimer;
  float magScale;

  bool magCalibration;
  float magCalibrationData[3][2];
  bool magCalibrationValid;

  VectorFloat magCalibrationScale;
  VectorFloat magCalibrationOffset;

  Timer fusionTimer;

  Timer telemetryTimer;
  bool telemetryUpdate;

  int16_t outputDisarmed[OUTPUT_CHANNELS];

  Stats stats;

  uint32_t modeMask;
  uint32_t modeMaskPrev;

  bool sensorCalibration;

  bool actuatorUpdate;
  uint32_t actuatorTimestamp;

  int16_t debug[4];

  BuzzerState buzzer;

  BatteryState battery;
};

}

#endif
