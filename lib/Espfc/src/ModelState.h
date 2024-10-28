#ifndef _ESPFC_MODEL_STATE_H_
#define _ESPFC_MODEL_STATE_H_

#include <Arduino.h>

#ifndef UNIT_TEST
#include <IPAddress.h>
#endif

#include "ModelConfig.h"
#include "Stats.h"
#include "helper_3dmath.h"
#include "Control/Pid.h"
#include "Kalman.h"
#include "Filter.h"
#include "Stats.h"
#include "Timer.h"
#include "Device/SerialDevice.h"
#include "Math/FreqAnalyzer.h"
#include "Msp/Msp.h"

namespace Espfc {

static const size_t CLI_BUFF_SIZE = 128;
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
      if(empty()) return BUZZER_SILENCE;
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
      if(voltage < 2.0) return false; // no battery connected
      return !samples && cellVoltage < vbatCellWarning * 0.01f;
    }

    int16_t rawVoltage;
    int16_t rawCurrent;
    float voltage;
    float voltageUnfiltered;
    float current;
    float currentUnfiltered;
    float cellVoltage;
    float percentage;
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
  FC_FAILSAFE_IDLE = 0,
  FC_FAILSAFE_RX_LOSS_DETECTED,
  FC_FAILSAFE_LANDING,
  FC_FAILSAFE_LANDED,
  FC_FAILSAFE_RX_LOSS_MONITORING,
  FC_FAILSAFE_RX_LOSS_RECOVERED
};

class FailsafeState
{
  public:
    FailsafePhase phase;
    uint32_t timeout;
};

constexpr float ACCEL_G = 9.80665f;
constexpr float ACCEL_G_INV = 1.f / ACCEL_G;

enum RescueConfigMode {
  RESCUE_CONFIG_PENDING,
  RESCUE_CONFIG_ACTIVE,
  RESCUE_CONFIG_DISABLED,
};

struct OutputTelemetryState
{
  int16_t errors[OUTPUT_CHANNELS];
  int32_t errorsSum[OUTPUT_CHANNELS];
  int32_t errorsCount[OUTPUT_CHANNELS];

  uint32_t erpm[OUTPUT_CHANNELS];
  float rpm[OUTPUT_CHANNELS];
  float freq[OUTPUT_CHANNELS];

  int8_t temperature[OUTPUT_CHANNELS];
  int8_t voltage[OUTPUT_CHANNELS];
  int8_t current[OUTPUT_CHANNELS];
  int8_t debug1[OUTPUT_CHANNELS];
  int8_t debug2[OUTPUT_CHANNELS];
  int8_t debug3[OUTPUT_CHANNELS];
  int8_t events[OUTPUT_CHANNELS];
};

struct OutputState
{
  float ch[OUTPUT_CHANNELS];
  int16_t us[OUTPUT_CHANNELS];
  int16_t disarmed[OUTPUT_CHANNELS];
  bool saturated;
  OutputTelemetryState telemetry;
};

struct InputState
{
  size_t channelCount;
  bool channelsValid;
  bool rxLoss;
  bool rxFailSafe;

  uint32_t frameTime;
  uint32_t frameDelta;
  uint32_t frameRate;
  uint32_t frameCount;
  uint32_t lossTime;

  float interpolationDelta;
  float interpolationStep;
  float autoFactor;
  float autoFreq;

  int16_t raw[INPUT_CHANNELS];
  int16_t buffer[INPUT_CHANNELS];
  int16_t bufferPrevious[INPUT_CHANNELS];

  float us[INPUT_CHANNELS];
  float ch[INPUT_CHANNELS];

  Filter filter[AXIS_COUNT_RPYT];

  Timer timer;
};

struct MixerState
{
  Timer timer;
  float minThrottle;
  float maxThrottle;
  bool digitalOutput;

  EscDriver * escMotor;
  EscDriver * escServo;
};

// working data
struct ModelState
{
  Device::GyroDevice* gyroDev;
  Device::MagDevice* magDev;
  Device::BaroDevice* baroDev;

  VectorInt16 gyroRaw;
  VectorFloat gyroSampled;
  VectorFloat gyroScaled;
  VectorFloat gyroDynNotch;
  VectorFloat gyroImu;

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

  bool imuUpdate;
  bool loopUpdate;
  VectorFloat pose;
  Quaternion poseQ;

  VectorFloat angle;
  Quaternion angleQ;

  RotationMatrixFloat boardAlignment;

  Filter gyroFilter[3];
  Filter gyroFilter2[3];
  Filter gyroFilter3[3];
  Filter gyroNotch1Filter[3];
  Filter gyroNotch2Filter[3];
  Filter gyroDynNotchFilter[6][3];
  Filter gyroImuFilter[3];

  Filter accelFilter[3];
  Filter magFilter[3];
  Filter rpmFreqFilter[RPM_FILTER_MOTOR_MAX];
  Filter rpmFilter[RPM_FILTER_MOTOR_MAX][RPM_FILTER_HARMONICS_MAX][3];

  VectorFloat velocity;
  VectorFloat desiredVelocity;

  VectorFloat desiredAngle;
  Quaternion desiredAngleQ;

  float desiredRate[AXES];

  Control::Pid innerPid[AXES];
  Control::Pid outerPid[AXES];

  InputState input;
  FailsafeState failsafe;

  MixerState mixer;
  OutputState output;

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
  int gyroCalibrationRate;

  int32_t gyroClock = 1000;
  int32_t gyroRate;

  Timer gyroTimer;
  Timer dynamicFilterTimer;

  Timer accelTimer;

  int32_t loopRate;
  Timer loopTimer;

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
  
  Timer telemetryTimer;

  Stats stats;

  uint32_t modeMask;
  uint32_t modeMaskPrev;
  uint32_t modeMaskSwitch;
  uint32_t modeMaskPresent;
  uint32_t disarmReason;

  bool airmodeAllowed;

  int16_t debug[8];

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
  float baroAltitudeRaw;
  float baroAltitude;
  float baroAltitudeBias;
  int32_t baroAltitudeBiasSamples;
  int32_t baroRate;

  uint32_t armingDisabledFlags;

  RescueConfigMode rescueConfigMode;

  SerialPortState serial[SERIAL_UART_COUNT];
  Timer serialTimer;

  Target::Queue appQueue;
};

}

#endif
