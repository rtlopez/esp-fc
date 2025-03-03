#ifndef _ESPFC_MODEL_STATE_H_
#define _ESPFC_MODEL_STATE_H_

#ifndef UNIT_TEST
#include <IPAddress.h>
#endif

#include "ModelConfig.h"
#include "helper_3dmath.h"
#include "Control/Pid.h"
#include "Kalman.h"
#include "Utils/Filter.h"
#include "Utils/Timer.h"
#include "Utils/Stats.h"
#include "Device/SerialDevice.h"
#include "Connect/Msp.hpp"
#include "Connect/StatusLed.hpp"

namespace Espfc {

constexpr size_t DEBUG_VALUE_COUNT = 8;
constexpr size_t CLI_BUFF_SIZE = 128;
constexpr size_t CLI_ARGS_SIZE = 12;

class CliCmd
{
  public:
    CliCmd(): buff{0}, index{0} {
      std::fill_n(args, CLI_ARGS_SIZE, nullptr);
    }
    const char * args[CLI_ARGS_SIZE];
    char buff[CLI_BUFF_SIZE];
    size_t index;
};

class SerialPortState
{
  public:
    Connect::MspMessage mspRequest;
    Connect::MspResponse mspResponse;
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

    Utils::Timer timer;
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
    Utils::Timer timer;
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

  Utils::Filter filter[AXIS_COUNT_RPYT];

  Utils::Timer timer;
};

struct MixerState
{
  Utils::Timer timer;
  float minThrottle;
  float maxThrottle;
  bool digitalOutput;

  EscDriver * escMotor;
  EscDriver * escServo;
};

struct MagState
{
  Device::MagDevice* dev;
  bool present;
  int rate;

  VectorInt16 raw;
  VectorFloat adc;
  Utils::Filter filter[3];
  Utils::Timer timer;

  int calibrationSamples;
  int calibrationState;
  bool calibrationValid;
  VectorFloat calibrationMin;
  VectorFloat calibrationMax;
  VectorFloat calibrationScale;
  VectorFloat calibrationOffset;

  VectorFloat pose;
};

struct BaroState
{
  Device::BaroDevice* dev;
  bool present;
  int32_t rate;

  float temperatureRaw;
  float temperature;
  float pressureRaw;
  float pressure;
  float altitudeRaw;
  float altitude;
  float altitudeBias;
  int32_t altitudeBiasSamples;
};

struct GyroState
{
  Device::GyroDevice* dev;
  bool present;
  int32_t rate;
  int32_t clock = 1000;

  VectorInt16 raw;
  VectorFloat adc;
  VectorFloat sampled;
  VectorFloat scaled;
  VectorFloat dynNotch;

  float scale;
  VectorFloat bias;
  float biasAlpha;
  int biasSamples;
  int calibrationState;
  int calibrationRate;

  Utils::Filter filter[AXIS_COUNT_RPY];
  Utils::Filter filter2[AXIS_COUNT_RPY];
  Utils::Filter filter3[AXIS_COUNT_RPY];
  Utils::Filter notch1Filter[AXIS_COUNT_RPY];
  Utils::Filter notch2Filter[AXIS_COUNT_RPY];
  Utils::Filter dynNotchFilter[DYN_NOTCH_COUNT_MAX][AXIS_COUNT_RPY];
  Utils::Filter rpmFilter[RPM_FILTER_MOTOR_MAX][RPM_FILTER_HARMONICS_MAX][AXIS_COUNT_RPY];
  Utils::Filter rpmFreqFilter[RPM_FILTER_MOTOR_MAX];

  Utils::Timer timer;
  Utils::Timer dynamicFilterTimer;
};

struct AccelState
{
  bool present;
  VectorInt16 raw;
  VectorFloat adc;
  VectorFloat prev;
  Utils::Filter filter[AXIS_COUNT_RPY];
  Utils::Timer timer;

  float scale;
  VectorFloat bias;
  float biasAlpha;
  int biasSamples;
  int calibrationState;
};

struct AttitudeState
{
  VectorFloat rate;
  Utils::Filter filter[AXIS_COUNT_RPY];
  VectorFloat euler;
  Quaternion quaternion;
};

struct SetpointState
{
  VectorFloat angle;
  float rate[AXIS_COUNT_RPYT];
};

struct ModeState
{
  uint32_t mask;
  uint32_t maskPrev;
  uint32_t maskSwitch;
  uint32_t maskPresent;
  uint32_t disarmReason;
  uint32_t armingDisabledFlags;
  RescueConfigMode rescueConfigMode;
  bool airmodeAllowed;
  uint32_t button;
  bool isSingleClickActive() const { return button & (1 << 0); }
  bool isDoubleClickActive() const { return button & (1 << 1); }
  bool isLongClickActive()   const { return button & (1 << 2); }
};

struct VtxState
{
  uint8_t active = false;
};

enum GpsDeviceVersion
{
  GPS_UNKNOWN,
  GPS_M8,
  GPS_M9,
  GPS_F9,
};

struct GpsSupportState
{
  GpsDeviceVersion version = GPS_UNKNOWN;
  bool glonass = false;
  bool galileo = false;
  bool beidou = false;
  bool sbas = false;
};

template<typename T>
struct GpsCoordinate
{
  T lat = T{}; // deg * 1e7
  T lon = T{}; // deg * 1e7
  T height = T{}; // mm (1e3)
};

struct GpsPosition
{
  GpsCoordinate<int32_t> raw;
  GpsCoordinate<int32_t> home;
};

template<typename T>
struct GpsSpeed
{
  T north = T{}; // mm/s (1e3)
  T east = T{}; // mm/s (1e3)
  T down = T{}; // mm/s (1e3)
  T groundSpeed = T{}; // mm/s (1e3)
  T heading = T{}; // deg * 1e5
  T speed3d = T{}; // mm/s (1e3)
};

struct GpsVelocity
{
  GpsSpeed<int32_t> raw;
};

struct GpsAccuracy
{
  uint32_t horizontal = 0; // mm (1e3)
  uint32_t vertical = 0; // mm (1e3)
  uint32_t speed = 0; // mm/s (1e3)
  uint32_t heading = 0; // deg * 1e5
  uint32_t pDop = 0; // (1e2)
};

struct GpsSatelite
{
  uint8_t gnssId = 0;
  uint8_t id = 0;
  uint8_t cno = 0;
  union {
    uint32_t value;
    struct {
      uint8_t qualityInd: 3; // quality indicatopr: 0-no signal, 1-searching, 2-aquired, 3-unstable, 4-code locked, 5,6,7-code and carrier locked
      uint8_t svUsed: 1; // used for navigation
      uint8_t health: 2; // signal health 0-unknown, 1-healthy, 2-unhealty
      uint8_t difCorr: 1; // differential correction available for this SV
      uint8_t smoothed: 1; // carrier smotthed pseudorange used
      uint8_t orbitSource: 3; // orbit source: 0-no inform, 1-ephemeris, 2-almanac, 3-assistnow offline, 4-assistnow autonomous, 5,6,7-other
      uint8_t ephAvail: 1; // ephemeris available
      uint8_t elmAvail: 1; // almanac available
      uint8_t enoAvail: 1; // assistnow offline available
      uint8_t eopAvail: 1; // assistnow autonomous available
      uint8_t reserved: 1;
      uint8_t sbasCorrUsed: 1; // SBAS corrections used
      uint8_t rtcmCorrUsed: 1; // RTCM corrections used
      uint8_t slasCorrUsed: 1; // SLAS corrections used
      uint8_t spartnCorrUsed: 1; // SPARTN corrections used
      uint8_t prCorrUsed: 1; // Pseudorange corrections used
      uint8_t crCorrUsed: 1; // Carrier range corrections used
      uint8_t doCorrUsed: 1; // Range rate (Doppler) corrections used
      uint8_t clasCorrUsed: 1; // CLAS corrections used
    };
  } quality = { .value = 0 };
};

struct GpsDateTime
{
  uint16_t year; // full year
  uint8_t month; // 1-12
  uint8_t day; // 1-31
  uint8_t hour; // 0-23
  uint8_t minute; // 0-59
  uint8_t second; // 0-59
  uint16_t msec; // 0-999
};

static constexpr size_t SAT_MAX = 32u;

struct GpsState
{
  bool fix = 0;
  uint8_t fixType = 0;
  uint8_t numSats = 0;
  uint8_t numCh = 0;
  bool present = false;
  bool frameError = false;
  bool wasLocked = false;
  bool homeSet = false;
  uint32_t interval;
  uint32_t lastMsgTs;
  GpsSupportState support;
  GpsPosition location;
  GpsVelocity velocity;
  GpsAccuracy accuracy;
  GpsDateTime dateTime;
  GpsSatelite svinfo[SAT_MAX];
};

// runtime data
struct ModelState
{
  GyroState gyro;
  AccelState accel;
  MagState mag;
  BaroState baro;
  GpsState gps;

  InputState input;
  FailsafeState failsafe;

  AttitudeState attitude;
  RotationMatrixFloat boardAlignment;

  SetpointState setpoint;
  Control::Pid innerPid[AXIS_COUNT_RPYT];
  Control::Pid outerPid[AXIS_COUNT_RPYT];

  MixerState mixer;
  OutputState output;
  VtxState vtx;

  int32_t loopRate;
  Utils::Timer loopTimer;

  Utils::Timer actuatorTimer;
  Utils::Timer telemetryTimer;

  ModeState mode;
  Utils::Stats stats;

  int16_t debug[DEBUG_VALUE_COUNT];

  BuzzerState buzzer;
  Connect::StatusLed led;

  BatteryState battery;

  MixerConfig currentMixer;
  MixerConfig customMixer;

  int16_t i2cErrorCount;
  int16_t i2cErrorDelta;

  SerialPortState serial[SERIAL_UART_COUNT];
  Utils::Timer serialTimer;

  Target::Queue appQueue;

  // other state
  Kalman kalman[AXIS_COUNT_RPYT];
  VectorFloat gyroPose;
  Quaternion gyroPoseQ;
  VectorFloat accelPose;
  VectorFloat accelPose2;
  Quaternion accelPoseQ;
  VectorFloat pose;
  Quaternion poseQ;
};

}

#endif
