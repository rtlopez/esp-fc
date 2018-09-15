#ifndef _ESPFC_MODEL_STATE_H_
#define _ESPFC_MODEL_STATE_H_

#include <Arduino.h>
#include <IPAddress.h>

#include "ModelConfig.h"
#include "Stats.h"
#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "Filter.h"
#include "Stats.h"
#include "Timer.h"
#include "SerialDevice.h"
#include "Math/FreqAnalyzer.h"

namespace Espfc {

static const size_t MSP_BUF_SIZE = 192;

enum MspState
{
  MSP_STATE_IDLE,
  MSP_STATE_HEADER_START,
  MSP_STATE_HEADER_M,
  MSP_STATE_HEADER_ARROW,
  MSP_STATE_HEADER_SIZE,
  MSP_STATE_HEADER_CMD,
  MSP_STATE_RECEIVED
};

enum MspType
{
  MSP_TYPE_CMD,
  MSP_TYPE_REPLY
};

class MspMessage
{
  public:
    MspMessage(): state(MSP_STATE_IDLE), expected(0), received(0), read(0) {}
    MspState state;
    MspType dir;
    uint8_t cmd;
    uint8_t expected;
    uint8_t received;
    uint8_t read;
    uint8_t buffer[MSP_BUF_SIZE];
    uint8_t checksum;

    int remain()
    {
      return received - read;
    }

    void advance(size_t size)
    {
      read += size;
    }

    uint8_t readU8()
    {
      return buffer[read++];
    }

    uint16_t readU16()
    {
      uint16_t ret;
      ret = readU8();
      ret |= readU8() << 8;
      return ret;
    }

    uint32_t readU32()
    {
      uint32_t ret;
      ret = readU8();
      ret |= readU8() <<  8;
      ret |= readU8() << 16;
      ret |= readU8() << 24;
      return ret;
    }
};

class MspResponse
{
  public:
    MspResponse(): len(0) {}
    uint8_t cmd;
    int8_t  result;
    uint8_t direction;
    uint8_t len;
    uint8_t data[MSP_BUF_SIZE];

    void writeData(const char * v, int size)
    {
      while(size-- > 0) writeU8(*v++);
    }

    void writeString(const char * v)
    {
      while(*v) writeU8(*v++);
    }

    void writeString(const __FlashStringHelper *ifsh)
    {
      PGM_P p = reinterpret_cast<PGM_P>(ifsh);
      while(true)
      {
        uint8_t c = pgm_read_byte(p++);
        if (c == 0) break;
        writeU8(c);
      }
    }

    void writeU8(uint8_t v)
    {
      data[len++] = v;
    }

    void writeU16(uint16_t v)
    {
      writeU8(v >> 0);
      writeU8(v >> 8);
    }

    void writeU32(uint32_t v)
    {
      writeU8(v >> 0);
      writeU8(v >> 8);
      writeU8(v >> 16);
      writeU8(v >> 24);
    }
};

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
    MspMessage mspRequest;
    MspResponse mspResponse;
    CliCmd cliCmd;
    SerialDevice * stream;
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

class MixerConfig {
  public:
    MixerConfig(): count(0), mixes(NULL) {}
    MixerConfig(const MixerConfig& c): count(c.count), mixes(c.mixes) {}
    MixerConfig(int8_t c, MixerEntry * m): count(c), mixes(m) {}

    int8_t count;
    MixerEntry * mixes;
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
  Filter gyroFilterImu[3];
  Math::FreqAnalyzer gyroAnalyzer[3];
  
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
  int16_t outputDisarmed[OUTPUT_CHANNELS];

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

  int32_t gyroClock;
  int32_t gyroRate;
  int32_t gyroDivider;

  Timer gyroTimer;

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
  bool telemetryUpdate;

  Stats stats;

  uint32_t modeMask;
  uint32_t modeMaskNew;
  uint32_t modeMaskPrev;

  bool airmodeAllowed;

  bool sensorCalibration;

  bool actuatorUpdate;
  uint32_t actuatorTimestamp;

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

  ArmingDisabledFlags armingDisabledFlags;

  IPAddress localIp;

  SerialPortState serial[SERIAL_UART_COUNT];
  Timer serialTimer;
};

}

#endif
