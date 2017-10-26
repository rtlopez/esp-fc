#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "EEPROM.h"
#include "Filter.h"
#include "Stats.h"
#include "Logger.h"
#include "Math.h"

#if 0
#define PIN_DEBUG(v) digitalWrite(D0, v)
#define PIN_DEBUG_INIT(v) pinMode(D0, v)
#else
#define PIN_DEBUG(v)
#define PIN_DEBUG_INIT(v)
#endif

namespace Espfc {

enum GyroRate {
  GYRO_RATE_1000 = 0x00,
  GYRO_RATE_500 = 0x01,
  GYRO_RATE_333 = 0x02,
  GYRO_RATE_250 = 0x03,
  GYRO_RATE_200 = 0x04,
  GYRO_RATE_166 = 0x05,
  GYRO_RATE_100 = 0x06,
  GYRO_RATE_50  = 0x07
};

enum GyroDlpf {
  GYRO_DLPF_256 = 0x00,
  GYRO_DLPF_188 = 0x01,
  GYRO_DLPF_98  = 0x02,
  GYRO_DLPF_42  = 0x03,
  GYRO_DLPF_20  = 0x04,
  GYRO_DLPF_10  = 0x05,
  GYRO_DLPF_5   = 0x06
};

enum GyroGain {
  GYRO_FS_250  = 0x00,
  GYRO_FS_500  = 0x01,
  GYRO_FS_1000 = 0x02,
  GYRO_FS_2000 = 0x03
};

enum AccelGain {
  ACCEL_FS_2  = 0x00,
  ACCEL_FS_4  = 0x01,
  ACCEL_FS_8  = 0x02,
  ACCEL_FS_16 = 0x03
};

enum AccelMode {
  ACCEL_NONE      = 0x00,
  ACCEL_DELAYED   = 0x01,
  ACCEL_GYRO      = 0x02,
  ACCEL_GYRO_FIFO = 0x03
};

enum MagGain {
  MAG_GAIN_1370 = 0x00,
  MAG_GAIN_1090 = 0x01,
  MAG_GAIN_820  = 0x02,
  MAG_GAIN_660  = 0x03,
  MAG_GAIN_440  = 0x04,
  MAG_GAIN_390  = 0x05,
  MAG_GAIN_330  = 0x06,
  MAG_GAIN_220  = 0x07,
};

enum MagRate {
  MAG_RATE_3    = 0x00,
  MAG_RATE_7P5  = 0x01,
  MAG_RATE_15   = 0x02,
  MAG_RATE_30   = 0x03,
  MAG_RATE_75   = 0x04,
};

enum MagAvg {
  MAG_AVERAGING_1 = 0x00,
  MAG_AVERAGING_2 = 0x01,
  MAG_AVERAGING_4 = 0x02,
  MAG_AVERAGING_8 = 0x03
};

enum FusionMode {
  FUSION_NONE          = 0x00,
  FUSION_SIMPLE        = 0x01,
  FUSION_EXPERIMENTAL  = 0x02,
  FUSION_MADGWICK      = 0x03,
  FUSION_RTQF          = 0x04,
  FUSION_LERP          = 0x05,
  FUSION_COMPLEMENTARY = 0x06,
  FUSION_KALMAN        = 0x07
};

enum FlightMode {
  MODE_ARMED    = 0x00,
  MODE_ANGLE    = 0x01,
  MODE_AIRMODE  = 0x02,
};

enum ModelFrame {
  FRAME_UNCONFIGURED  = 0x00,
  FRAME_QUAD_X        = 0x01,
  FRAME_BALANCE_ROBOT = 0x02,
  FRAME_DIRECT        = 0x03,
  FRAME_GIMBAL        = 0x04
};

enum ActuatorConfig {
  ACT_INNER_P     = 1 << 0,
  ACT_INNER_I     = 1 << 1,
  ACT_INNER_D     = 1 << 2,
  ACT_OUTER_P     = 1 << 3,
  ACT_OUTER_I     = 1 << 4,
  ACT_OUTER_D     = 1 << 5,
  ACT_AXIS_ROLL   = 1 << 6,
  ACT_AXIS_PITCH  = 1 << 7,
  ACT_AXIS_YAW    = 1 << 8,
  ACT_AXIS_THRUST = 1 << 9,
  ACT_GYRO_THRUST = 1 << 10
};

enum DebugMode {
  DEBUG_NONE,
  DEBUG_CYCLETIME,
  DEBUG_BATTERY,
  DEBUG_GYRO,
  DEBUG_ACCELEROMETER,
  DEBUG_MIXER,
  DEBUG_AIRMODE,
  DEBUG_PIDLOOP,
  DEBUG_NOTCH,
  DEBUG_RC_INTERPOLATION,
  DEBUG_VELOCITY,
  DEBUG_DTERM_FILTER,
  DEBUG_ANGLERATE,
  DEBUG_ESC_SENSOR,
  DEBUG_SCHEDULER,
  DEBUG_STACK,
  DEBUG_ESC_SENSOR_RPM,
  DEBUG_ESC_SENSOR_TMP,
  DEBUG_ALTITUDE,
  DEBUG_FFT,
  DEBUG_FFT_TIME,
  DEBUG_FFT_FREQ,
  DEBUG_FRSKY_D_RX,
  DEBUG_COUNT
};

enum SerialSpeed {
  SERIAL_SPEED_NONE   =      0,
  SERIAL_SPEED_9600   =   9600,
  SERIAL_SPEED_19200  =  19200,
  SERIAL_SPEED_38400  =  38400,
  SERIAL_SPEED_57600  =  57600,
  SERIAL_SPEED_115200 = 115200,
  SERIAL_SPEED_230400 = 230400,
  SERIAL_SPEED_250000 = 250000,
  SERIAL_SPEED_500000 = 500000
};

enum SerialSpeedIndex {
  SERIAL_SPEED_INDEX_AUTO = 0,
  SERIAL_SPEED_INDEX_9600,
  SERIAL_SPEED_INDEX_19200,
  SERIAL_SPEED_INDEX_38400,
  SERIAL_SPEED_INDEX_57600,
  SERIAL_SPEED_INDEX_115200,
  SERIAL_SPEED_INDEX_230400,
  SERIAL_SPEED_INDEX_250000,
  SERIAL_SPEED_INDEX_500000 = 10
};

enum SerialPort {
  SERIAL_UART_NONE = 0,
  SERIAL_UART_1,
  SERIAL_UART_2,
  SERIAL_UART_COUNT
};

enum SerialFunction {
  SERIAL_FUNCTION_NONE                = 0,
  SERIAL_FUNCTION_MSP                 = (1 << 0),  // 1
  SERIAL_FUNCTION_GPS                 = (1 << 1),  // 2
  SERIAL_FUNCTION_TELEMETRY_FRSKY     = (1 << 2),  // 4
  SERIAL_FUNCTION_TELEMETRY_HOTT      = (1 << 3),  // 8
  SERIAL_FUNCTION_TELEMETRY_LTM       = (1 << 4),  // 16
  SERIAL_FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
  SERIAL_FUNCTION_RX_SERIAL           = (1 << 6),  // 64
  SERIAL_FUNCTION_BLACKBOX            = (1 << 7),  // 128
  SERIAL_FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
  SERIAL_FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
  SERIAL_FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
  SERIAL_FUNCTION_TELEMETRY_IBUS      = (1 << 12), // 4096
  SERIAL_FUNCTION_VTX_TRAMP           = (1 << 13), // 8192
  SERIAL_FUNCTION_RCDEVICE            = (1 << 14), // 16384
};


enum OutputProtocol {
  OUTPUT_PWM,
  OUTPUT_ONESHOT125
};

const size_t OUTPUT_CHANNELS = 4;

enum Axis {
  AXIS_ROLL,    // x
  AXIS_PITCH,   // y
  AXIS_YAW,     // z
  AXIS_THRUST,  // throttle channel index
  AXIS_AUX_1,
  AXIS_AUX_2,
  AXIS_AUX_3,
  AXIS_AUX_4,
  AXIS_COUNT
};

const size_t AXES            = 4;
const size_t INPUT_CHANNELS  = AXIS_COUNT;

enum PidIndex {
  PID_ROLL,
  PID_PITCH,
  PID_YAW,
  PID_ALT,
  PID_POS,
  PID_POSR,
  PID_NAVR,
  PID_LEVEL,
  PID_MAG,
  PID_VEL,
  PID_ITEM_COUNT
};

class ActuatorCondition
{
  public:
    uint8_t id;
    uint8_t ch;
    uint8_t min;
    uint8_t max;
};

#define ACTUATOR_CONDITIONS 8

// working data
struct ModelState
{
  VectorInt16 gyroRaw;
  VectorInt16 accelRaw;
  VectorInt16 magRaw;

  VectorFloat gyroScaled;
  VectorFloat accelScaled;
  VectorFloat magScaled;

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
  Filter accelFilter[3];
  Filter magFilter[3];

  VectorFloat velocity;
  VectorFloat desiredVelocity;

  VectorFloat controlAngle;
  VectorFloat desiredAngle;
  Quaternion desiredAngleQ;

  float desiredRate[AXES];

  Pid innerPid[AXES];
  Pid outerPid[AXES];

  short inputUs[INPUT_CHANNELS];
  float input[INPUT_CHANNELS];
  unsigned long inputDelay;
  float inputFilterAlpha;

  float output[OUTPUT_CHANNELS];
  short outputUs[OUTPUT_CHANNELS];

  // other state
  Kalman kalman[AXES];
  VectorFloat accelPrev;

  float accelScale;
  VectorFloat accelBias;
  float accelBiasAlpha;
  int accelBiasSamples;
  bool accelBiasValid;

  float gyroScale;
  VectorFloat gyroBias;
  float gyroBiasAlpha;
  long gyroBiasSamples;
  bool gyroBiasValid;

  float gyroDeadband;

  uint8_t gyroDivider;
  uint32_t gyroSampleRate;
  uint32_t gyroSampleInterval;
  float gyroDt;

  uint32_t gyroTimestamp;
  uint32_t gyroIteration;
  bool gyroUpdate;

  uint32_t loopSampleRate;
  uint32_t loopSampleInterval;
  uint32_t loopTimestamp;
  uint32_t loopIteration;
  float loopDt;
  bool loopUpdate;

  bool mixerUpdate;

  uint32_t magTimestamp;
  long magSampleInterval;
  long magSampleRate;
  float magScale;

  bool magCalibration;
  float magCalibrationData[3][2];
  bool magCalibrationValid;

  VectorFloat magCalibrationScale;
  VectorFloat magCalibrationOffset;

  uint32_t telemetryTimestamp;
  bool telemetryUpdate;

  uint16_t outputDisarmed[OUTPUT_CHANNELS];

  Stats stats;

  uint32_t modeMask;
};

// persistent data
struct ModelConfig
{
  int8_t gyroDlpf;
  int8_t gyroFsr;
  int8_t accelFsr;
  int8_t gyroSampleRate;
  int8_t gyroDeadband;
  int8_t accelMode;

  int8_t gyroFilterType;
  int16_t gyroFilterCutFreq;
  int8_t accelFilterType;
  int16_t accelFilterCutFreq;
  int8_t magFilterType;
  int16_t magFilterCutFreq;
  int8_t dtermFilterType;
  int16_t dtermFilterCutFreq;

  uint8_t dtermSetpointWeight;
  int8_t itermWindupPointPercent;

  bool magEnable;
  int8_t magSampleRate;
  int8_t magFsr;
  int8_t magAvr;

  int8_t modelFrame;

  int32_t actuatorConfig[3];
  int8_t actuatorChannels[3];
  int16_t actuatorMin[3];
  int16_t actuatorMax[3];

  ActuatorCondition conditions[ACTUATOR_CONDITIONS];

  int8_t ppmPin;
  int8_t ppmMode;
  int16_t inputMin[INPUT_CHANNELS];
  int16_t inputNeutral[INPUT_CHANNELS];
  int16_t inputMax[INPUT_CHANNELS];
  int8_t inputMap[INPUT_CHANNELS];
  int8_t inputDeadband;
  int8_t inputFilterAlpha;

  uint8_t inputExpo[3];
  uint8_t inputRate[3];
  uint8_t inputSuperRate[3];

  int16_t outputMin[OUTPUT_CHANNELS];
  int16_t outputNeutral[OUTPUT_CHANNELS];
  int16_t outputMax[OUTPUT_CHANNELS];

  int8_t outputPin[OUTPUT_CHANNELS];
  int8_t outputProtocol;
  int16_t outputAsync;
  int16_t outputRate;
  int8_t yawReverse;

  PidConfig pid[PID_ITEM_COUNT];

  int8_t angleLimit;
  int16_t angleRateLimit;

  int8_t loopSync;
  int8_t mixerSync;

  int16_t lowThrottleTreshold;
  bool lowThrottleZeroIterm;
  bool lowThrottleMotorStop;

  bool telemetry;
  int32_t telemetryInterval;
  int8_t telemetryPort;

  bool blackbox;
  int8_t blackboxPort;
  int8_t cliPort;

  int32_t uart1Speed;
  int32_t uart2Speed;

  int8_t fusionMode;
  bool fusionDelay;

  int16_t gyroBias[3];
  int16_t accelBias[3];
  int16_t magCalibrationScale[3];
  int16_t magCalibrationOffset[3];
};

class Model
{
  public:
    Model() {
      config.accelMode = ACCEL_GYRO;
      config.gyroDlpf = GYRO_DLPF_256;
      config.gyroFsr  = GYRO_FS_2000;
      config.accelFsr = ACCEL_FS_8;
      config.gyroSampleRate = GYRO_RATE_1000;
      config.gyroDeadband = 0;

      config.magEnable = 0;
      config.magSampleRate = MAG_RATE_75;
      config.magAvr = MAG_AVERAGING_1;

      config.loopSync = 1;
      config.mixerSync = 1;

      config.fusionMode = FUSION_MADGWICK;
      config.fusionDelay = 0;

      config.gyroFilterType = FILTER_PT1;
      config.gyroFilterCutFreq = 70;
      config.accelFilterType = FILTER_PT1;
      config.accelFilterCutFreq = 15;
      config.magFilterType = FILTER_PT1;
      config.magFilterCutFreq = 15;
      config.dtermFilterType = FILTER_BIQUAD;
      config.dtermFilterCutFreq = 50;

      config.uart1Speed = SERIAL_SPEED_115200;

      config.uart2Speed = SERIAL_SPEED_115200;
      //config.uart2Speed = SERIAL_SPEED_230400;
      //config.uart2Speed = SERIAL_SPEED_250000;
      //config.uart2Speed = SERIAL_SPEED_500000;
      //config.uart2Speed = SERIAL_SPEED_NONE;

      config.cliPort = SERIAL_UART_1;

      config.telemetry = 0;
      config.telemetryInterval = 1000 * 1000;
      config.telemetryPort = SERIAL_UART_1;

      config.blackbox = 0;
      //config.blackboxPort = SERIAL_UART_2;
      config.blackboxPort = SERIAL_UART_NONE;

      // output config
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        config.outputMin[i] = 1050;
        config.outputMax[i] = 1990;
        config.outputNeutral[i] = (config.outputMin[i] + config.outputMax[i]) / 2;
      }

      config.outputPin[0] = D3; // D8; // -1 off
      config.outputPin[1] = D5; // D6;
      config.outputPin[2] = D6; // D5;
      config.outputPin[3] = D8; // D3;

      config.modelFrame = FRAME_QUAD_X;
      config.yawReverse = 0;

      //config.modelFrame = FRAME_DIRECT;
      //config.outputProtocol = OUTPUT_PWM;
      config.outputProtocol = OUTPUT_ONESHOT125;
      config.outputRate = 500;    // max 500 for PWM, 2000 for Oneshot125
      config.outputAsync = false;

      // input config
      config.ppmPin = D7;     // GPIO13
      config.ppmMode = RISING;
      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        config.inputMap[i] = i;
      }
      // swap yaw and throttle
      config.inputMap[2] = 3; // replace input 2 with rx channel 3, yaw
      config.inputMap[3] = 2; // replace input 3 with rx channel 2, throttle

      for(size_t i = AXIS_ROLL; i <= AXIS_PITCH; i++)
      {
        config.inputRate[i] = 70;
        config.inputExpo[i] = 0;
        config.inputSuperRate[i] = 80;
      }
      config.inputRate[AXIS_YAW] = 120;
      config.inputExpo[AXIS_YAW] = 0;
      config.inputSuperRate[AXIS_YAW] = 50;

      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        config.inputMin[i] = 1000;
        config.inputNeutral[i] = 1500;
        config.inputMax[i] = 2000;
      }
      config.inputMin[AXIS_YAW] = 2000;        // invert Yaw axis
      config.inputMax[AXIS_YAW] = 1000;

      config.inputDeadband = 3; // us
      config.inputFilterAlpha = 100;

      // PID controller config
      config.pid[PID_ROLL]  = { .P = 45,  .I = 45, .D = 14 };
      config.pid[PID_PITCH] = { .P = 63,  .I = 54, .D = 18 };
      config.pid[PID_YAW]   = { .P = 125, .I = 75, .D = 5 };
      config.pid[PID_LEVEL] = { .P = 30,  .I = 0,  .D = 0 };

      config.itermWindupPointPercent = 30;
      config.dtermSetpointWeight = 50;

      config.angleLimit = 50;  // deg
      config.angleRateLimit = 300;  // deg

      config.lowThrottleTreshold = 1050;
      config.lowThrottleZeroIterm = true;
      config.lowThrottleMotorStop = true;

      config.conditions[0].id = MODE_ARMED;
      config.conditions[0].ch = 0; // aux1
      config.conditions[0].min = (1700 - 900) / 25;
      config.conditions[0].max = (2100 - 900) / 25;

      config.conditions[1].id = MODE_ANGLE;
      config.conditions[1].ch = 0; // aux1
      config.conditions[1].min = (1700 - 900) / 25;
      config.conditions[1].max = (2100 - 900) / 25;

      config.conditions[2].id = MODE_AIRMODE;
      config.conditions[2].ch = 0; // aux1
      config.conditions[2].min = (1200 - 900) / 25;
      config.conditions[2].max = (1700 - 900) / 25;

      // actuator config - pid scaling
      config.actuatorConfig[0] = ACT_INNER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      config.actuatorChannels[0] = 5;
      config.actuatorMin[0] = 25;
      config.actuatorMax[0] = 400;

      config.actuatorConfig[1] = ACT_INNER_I | ACT_AXIS_ROLL | ACT_AXIS_PITCH;
      config.actuatorChannels[1] = 6;
      config.actuatorMin[1] = 25;
      config.actuatorMax[1] = 400;

      config.actuatorConfig[2] = ACT_INNER_D | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      //config.actuatorConfig[2] = ACT_OUTER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      //config.actuatorConfig[2] = ACT_INNER_P | ACT_AXIS_YAW;
      config.actuatorChannels[2] = 7;
      config.actuatorMin[2] = 25;
      config.actuatorMax[2] = 400;

      //config.actuatorConfig[2] = ACT_GYRO_THRUST;
      //config.actuatorMin[2] = -0.05;
      //config.actuatorMax[2] =  0.05;

      // default callibration values
      config.gyroBias[0] = 0;
      config.gyroBias[1] = 0;
      config.gyroBias[2] = 0;
      config.accelBias[0] = 0;
      config.accelBias[1] = 0;
      config.accelBias[2] = 0;
      config.magCalibrationOffset[0] = 0;
      config.magCalibrationOffset[1] = 0;
      config.magCalibrationOffset[2] = 0;
      config.magCalibrationScale[0] = 1000;
      config.magCalibrationScale[1] = 1000;
      config.magCalibrationScale[2] = 1000;
    }

    bool isMode(FlightMode mode)
    {
      return state.modeMask & (1 << mode);
    }

    void calibrate()
    {
      state.gyroBiasSamples  = 2 * state.gyroSampleRate;
      state.accelBiasSamples = 2 * state.gyroSampleRate;
    }

    void begin()
    {
      logger.begin();
      EEPROM.begin(512);
      load();
      update();
    }

    void update()
    {
      // ensure disarmed pulses
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        state.outputDisarmed[i] = 1000;
      }

      // update initial sensor calibration
      for(size_t i = 0; i < 3; i++)
      {
        state.gyroBias.set(i, config.gyroBias[i] / 1000.0f);
        state.accelBias.set(i, config.accelBias[i] / 1000.0f);
        state.magCalibrationOffset.set(i, config.magCalibrationOffset[i] / 1000.0f);
        state.magCalibrationScale.set(i, config.magCalibrationScale[i] / 1000.0f);
      }

      state.gyroDeadband = radians(config.gyroDeadband / 10.0f);
      state.inputFilterAlpha = Math::bound((config.inputFilterAlpha / 100.0f), 0.f, 1.f);

      for(size_t i = 0; i < 3; i++) // rpy
      {
        PidConfig * pc = &config.pid[i];
        state.innerPid[i].configurePid(
          pc->P * PTERM_SCALE,
          pc->I * ITERM_SCALE,
          pc->D * DTERM_SCALE,
          config.itermWindupPointPercent / 100.0f,
          config.dtermSetpointWeight / 100.0f,
          1.0f
        );
      }

      for(size_t i = 0; i < 3; i++)
      {
        PidConfig * pc = &config.pid[PID_LEVEL];
        state.outerPid[i].configurePid(
          pc->P * LEVEL_PTERM_SCALE,
          pc->I * LEVEL_ITERM_SCALE,
          pc->D * LEVEL_DTERM_SCALE,
          0.1f,
          0,
          radians(config.angleRateLimit)
        );
      }
    }

    void preSave()
    {
      // store current sensor calibration
      for(size_t i = 0; i < 3; i++)
      {
        config.gyroBias[i] = lrintf(state.gyroBias[i] * 1000.0f);
        config.accelBias[i] = lrintf(state.accelBias[i] * 1000.0f);
        config.magCalibrationOffset[i] = lrintf(state.magCalibrationOffset[i] * 1000.0f);
        config.magCalibrationScale[i] = lrintf(state.magCalibrationScale[i] * 1000.0f);
      }
    }

    ModelState state;
    ModelConfig config;
    Logger logger;

    int load()
    {
      int addr = 0;
      uint8_t magic = EEPROM.read(addr++);
      if(EEPROM_MAGIC != magic)
      {
        logger.err().log(F("eeprom bad magic"));
        return -1;
      }

      uint8_t version = EEPROM.read(addr++);
      if(version != EEPROM_VERSION)
      {
        logger.err().log(F("eeprom wrong version"));
        return -1;
      }

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config);
      uint8_t * end = begin + sizeof(ModelConfig);
      for(uint8_t * it = begin; it < end; ++it)
      {
        *it = EEPROM.read(addr++);
      }
      logger.info().logln(F("eeprom load"));
      return 1;
    }

    void save()
    {
      preSave();
      int addr = 0;
      EEPROM.write(addr++, EEPROM_MAGIC);
      EEPROM.write(addr++, EEPROM_VERSION);

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config);
      uint8_t * end = begin + sizeof(ModelConfig);
      for(uint8_t * it = begin; it < end; ++it)
      {
        EEPROM.write(addr++, *it);
      }
      EEPROM.commit();
      logger.info().logln(F("eeprom save"));
    }

    void reset()
    {
      config = ModelConfig();
      //update();
    }

    static const uint8_t EEPROM_MAGIC   = 0xA5;
    static const uint8_t EEPROM_VERSION = 0x02;
};

}

#endif
