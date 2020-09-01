#ifndef _ESPFC_MODEL_STATE_H_
#define _ESPFC_MODEL_STATE_H_

#include <Arduino.h>

#ifndef UNIT_TEST
#include <IPAddress.h>
#endif

#include "ModelConfig.h"
#include "Stats.h"
#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "Filter.h"
#include "Stats.h"
#include "Timer.h"
#include "Device/SerialDevice.h"
#include "Math/FreqAnalyzer.h"
#include "Msp/Msp.h"

namespace Espfc {

static const size_t CLI_BUFF_SIZE = 64;
static const size_t CLI_ARGS_SIZE = 12;

class CliCmd
{
  public:
    CliCmd(): buff{0}, index(0) { for(size_t i = 0; i < CLI_ARGS_SIZE; ++i) args[i] = nullptr; }
    const char * args[CLI_ARGS_SIZE];
    char buff[CLI_BUFF_SIZE];
    size_t index;
};

class SerialPortState
{
  public:
    Msp::MspMessage mspRequest;
    Msp::MspResponse mspResponse;
    CliCmd cliCmd;
    Device::SerialDevice * stream;
    uint32_t availableFrom;
};

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

enum CalibrationState {
  CALIBRATION_IDLE   = 0,
  CALIBRATION_START  = 1,
  CALIBRATION_UPDATE = 2,
  CALIBRATION_APPLY  = 3,
  CALIBRATION_SAVE   = 4,
};

enum FailsafePhase {
  FAILSAFE_IDLE = 0,
  FAILSAFE_RX_LOSS_DETECTED,
  FAILSAFE_LANDING,
  FAILSAFE_LANDED,
  FAILSAFE_RX_LOSS_MONITORING,
  FAILSAFE_RX_LOSS_RECOVERED
};

class FailsafeState
{
  public:
    FailsafePhase phase;
    uint32_t timeout;
};

#define ACCEL_G (9.80665f)
#define ACCEL_G_INV (1.f / ACCEL_G)

// working data
struct ModelState
{
  VectorInt16 gyroRaw;
  VectorInt16 accelRaw;
  VectorInt16 magRaw;

  VectorFloat gyro;
  VectorFloat accel;
  VectorFloat mag;

  VectorFloat gyroImu;

  VectorFloat gyroPose;
  Quaternion gyroPoseQ;
  VectorFloat accelPose;
  VectorFloat accelPose2;
  Quaternion accelPoseQ;
  VectorFloat magPose;

  VectorFloat pose;
  Quaternion poseQ;

  VectorFloat angle;
  Quaternion angleQ;

  Filter gyroFilter[3];
  Filter gyroFilter2[3];
  Filter gyroFilter3[3];
  Filter gyroNotch1Filter[3];
  Filter gyroNotch2Filter[3];
  Filter gyroDynamicFilter[3];
  Filter gyroDynamicFilter2[3];
  Filter gyroFilterImu[3];
  Math::FreqAnalyzer gyroAnalyzer[3];
  
  Filter accelFilter[3];
  Filter magFilter[3];
  Filter inputFilter[4];

  VectorFloat velocity;
  VectorFloat desiredVelocity;

  VectorFloat desiredAngle;
  Quaternion desiredAngleQ;

  float desiredRate[AXES];

  Pid innerPid[AXES];
  Pid outerPid[AXES];

  size_t inputChannelCount;
  bool inputChannelsValid;
  bool inputRxLoss;
  bool inputRxFailSafe;

  uint32_t inputFrameTime;
  uint32_t inputFrameDelta;
  uint32_t inputFrameRate;
  uint32_t inputFrameCount;
  float inputInterpolationDelta;
  float inputInterpolationRate;

  int16_t inputRaw[INPUT_CHANNELS];
  int16_t inputBuffer[INPUT_CHANNELS];
  int16_t inputBufferPrevious[INPUT_CHANNELS];

  float inputUs[INPUT_CHANNELS];
  float input[INPUT_CHANNELS];
  float inputPrevious[INPUT_CHANNELS];
  float inputDelta[INPUT_CHANNELS];
  FailsafeState failsafe;

  float output[OUTPUT_CHANNELS];
  int16_t outputUs[OUTPUT_CHANNELS];
  int16_t outputDisarmed[OUTPUT_CHANNELS];

  // other state
  Kalman kalman[AXES];
  VectorFloat accelPrev;

  float accelScale;
  VectorFloat accelBias;
  float accelBiasAlpha;
  int accelBiasSamples;
  int accelCalibrationState;

  float gyroScale;
  VectorFloat gyroBias;
  float gyroBiasAlpha;
  int gyroBiasSamples;
  int gyroCalibrationState;

  int32_t gyroClock;
  int32_t gyroRate;
  int32_t gyroDivider;

  Timer gyroTimer;
  Timer dynamicFilterTimer;

  Timer accelTimer;

  int32_t loopRate;
  Timer loopTimer;

  Timer mixerTimer;
  float minThrottle;
  float maxThrottle;
  bool digitalOutput;

  Timer actuatorTimer;

  Timer magTimer;
  int magRate;

  int magCalibrationSamples;
  int magCalibrationState;
  bool magCalibrationValid;

  VectorFloat magCalibrationMin;
  VectorFloat magCalibrationMax;
  VectorFloat magCalibrationScale;
  VectorFloat magCalibrationOffset;
  
  bool telemetry;
  Timer telemetryTimer;

  Stats stats;

  uint32_t modeMask;
  uint32_t modeMaskPrev;
  uint32_t modeMaskSwitch;

  bool airmodeAllowed;

  int16_t debug[4];

  BuzzerState buzzer;

  BatteryState battery;

  MixerConfig currentMixer;
  MixerConfig customMixer;

  int16_t i2cErrorCount;
  int16_t i2cErrorDelta;

  bool gyroPresent;
  bool accelPresent;
  bool magPresent;
  bool baroPresent;
  
  float baroTemperatureRaw;
  float baroTemperature;
  float baroPressureRaw;
  float baroPressure;
  float baroAltitude;
  float baroAltitudeBias;
  int32_t baroAlititudeBiasSamples;

  uint32_t armingDisabledFlags;

  IPAddress localIp;

  SerialPortState serial[SERIAL_UART_COUNT];
  Timer serialTimer;
};

}

#endif
