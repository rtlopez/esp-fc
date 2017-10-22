#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "EEPROM.h"
#include "Filter.h"
#include "Stats.h"
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
  MODE_OFF      = 0x00,
  MODE_DIRECT   = 0x01,
  MODE_RATE     = 0x02,
  MODE_ANGLE    = 0x03,
  MODE_ANGLE_SIMPLE    = 0x05,
  MODE_BALANCING_ROBOT = 0x06,
  MODE_BALANCING_ANGLE = 0x07
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

enum SerialPort {
  SERIAL_UART_NONE = 0,
  SERIAL_UART_1,
  SERIAL_UART_2
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

  short flightMode;
  bool armed;

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
  float gyroScale;
  VectorFloat gyroBias;
  float gyroBiasAlpha;
  long gyroBiasSamples;
  bool gyroBiasValid;
  uint8_t gyroDivider;
  float gyroDeadband;

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

  uint32_t telemetryTimestamp;
  bool telemetryUpdate;

  uint16_t outputDisarmed[OUTPUT_CHANNELS];

  Stats stats;
};

// persistent data
struct ModelConfig
{
  uint8_t gyroDlpf;
  uint8_t gyroFsr;
  uint8_t accelFsr;
  uint8_t gyroSampleRate;
  uint8_t accelMode;

  uint8_t magSampleRate;
  uint8_t magFsr;
  uint8_t magAvr;

  uint8_t gyroDeadband;

  bool magEnable;
  VectorInt16 magCalibrationScale;
  VectorInt16 magCalibrationOffset;

  uint8_t gyroFilterType;
  uint16_t gyroFilterCutFreq;
  uint8_t accelFilterType;
  uint16_t accelFilterCutFreq;
  uint8_t magFilterType;
  uint16_t magFilterCutFreq;
  uint8_t dtermFilterType;
  uint16_t dtermFilterCutFreq;

  uint8_t dtermSetpointWeight;
  uint8_t itermWindupPointPercent;

  uint8_t modelFrame;
  uint8_t flightModeChannel;
  uint8_t flightModes[3];

  uint32_t actuatorConfig[3];
  uint8_t actuatorChannels[3];
  uint16_t actuatorMin[3];
  uint16_t actuatorMax[3];

  int8_t ppmPin;
  uint8_t ppmMode;
  uint16_t inputMin[INPUT_CHANNELS];
  uint16_t inputNeutral[INPUT_CHANNELS];
  uint16_t inputMax[INPUT_CHANNELS];
  uint8_t inputMap[INPUT_CHANNELS];
  uint8_t inputDeadband;
  uint8_t inputFilterAlpha;

  uint8_t inputExpo[3];
  uint8_t inputRate[3];
  uint8_t inputSuperRate[3];

  uint16_t outputMin[OUTPUT_CHANNELS];
  uint16_t outputNeutral[OUTPUT_CHANNELS];
  uint16_t outputMax[OUTPUT_CHANNELS];

  int8_t outputPin[OUTPUT_CHANNELS];
  uint8_t outputProtocol;
  uint16_t outputRate;

  PidConfig pid[PID_ITEM_COUNT];

  uint8_t angleMax;
  uint8_t velocityMax;
  uint8_t loopSync;
  uint8_t mixerSync;

  uint16_t lowThrottleTreshold;
  bool lowThrottleZeroIterm;
  bool lowThrottleMotorStop;

  bool telemetry;
  uint32_t telemetryInterval;
  uint8_t telemetryPort;

  bool blackbox;
  uint8_t blackboxPort;
  uint8_t cliPort;

  uint32_t uart1Speed;
  uint32_t uart2Speed;

  uint8_t fusionMode;
  bool fusionDelay;
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

      for(size_t i = 0; i < 3; i++)
      {
        config.magCalibrationOffset.set(i, 0.f);
        config.magCalibrationScale.set(i, 1.f);
      }

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
        config.outputNeutral[i] = 1520;
        config.outputMax[i] = 1990;
      }

      config.outputPin[0] = D3; // D8; // -1 off
      config.outputPin[1] = D5; // D6;
      config.outputPin[2] = D6; // D5;
      config.outputPin[3] = D8; // D3;

      config.modelFrame = FRAME_QUAD_X;
      //config.modelFrame = FRAME_DIRECT;
      config.outputRate = 500;    // 125 max for old driver
      //config.outputProtocol = OUTPUT_PWM;
      config.outputProtocol = OUTPUT_ONESHOT125;

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
        config.inputRate[i] = 100;
        config.inputExpo[i] = 0;
        config.inputSuperRate[i] = 60;
      }
      config.inputRate[AXIS_YAW] = 100;
      config.inputExpo[AXIS_YAW] = 0;
      config.inputSuperRate[AXIS_YAW] = 50;

      config.flightModeChannel = 4;
      config.flightModes[0] = MODE_OFF;
      config.flightModes[1] = MODE_ANGLE;
      config.flightModes[2] = MODE_RATE;

      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        config.inputMin[i] = 1000;
        config.inputNeutral[i] = 1500;
        config.inputMax[i] = 2000;
      }
      config.inputMin[AXIS_YAW] = 2000;        // invert Yaw axis
      config.inputMax[AXIS_YAW] = 1000;

      config.inputDeadband = 3; // us
      config.inputFilterAlpha = 80;

      // PID controller config
      config.pid[PID_PITCH] = { .P = 45,  .I = 45, .D = 14 };
      config.pid[PID_ROLL]  = { .P = 63,  .I = 54, .D = 18 };
      config.pid[PID_YAW]   = { .P = 125, .I = 75, .D = 5 };
      config.pid[PID_LEVEL] = { .P = 30,  .I = 0,  .D = 0 };

      float iwp = config.itermWindupPointPercent = 30;
      float dsw = config.dtermSetpointWeight = 50;

      config.angleMax = 50;  // deg

      config.lowThrottleTreshold = 1050;
      config.lowThrottleZeroIterm = true;
      config.lowThrottleMotorStop = true;

      // actuator config - pid scaling
      config.actuatorConfig[0] = ACT_INNER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      config.actuatorChannels[0] = 5;
      config.actuatorMin[0] = 0.25;
      config.actuatorMax[0] = 4;

      config.actuatorConfig[1] = ACT_INNER_I | ACT_AXIS_ROLL | ACT_AXIS_PITCH;
      config.actuatorChannels[1] = 6;
      config.actuatorMin[1] = 0.25;
      config.actuatorMax[1] = 4;

      config.actuatorConfig[2] = ACT_INNER_D | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      //config.actuatorConfig[2] = ACT_OUTER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      //config.actuatorConfig[2] = ACT_INNER_P | ACT_AXIS_YAW;
      config.actuatorChannels[2] = 7;
      config.actuatorMin[2] = 0.25;
      config.actuatorMax[2] = 4;

      //config.actuatorConfig[2] = ACT_GYRO_THRUST;
      //config.actuatorMin[2] = -0.05;
      //config.actuatorMax[2] =  0.05;
    }

    void begin()
    {
      load();
      update();
    }

    void update()
    {
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        state.outputDisarmed[i] = 1000;
      }
      
      state.gyroDeadband = radians(config.gyroDeadband / 10.0f);
      state.inputFilterAlpha = Math::bound((config.inputFilterAlpha / 100.0f), 0.f, 1.f);

      for(size_t i = 0; i < 3; i++)
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
          radians(300)
        );
      }
    }

    ModelState state;
    ModelConfig config;

    int load()
    {
      int addr = 0;
      uint8_t magic = EEPROM.read(addr++);
      if(EEPROM_MAGIC != magic) return -1;

      uint8_t version = EEPROM.read(addr++);

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config);
      uint8_t * end = begin + sizeof(ModelConfig);
      for(uint8_t * it = begin; it < end; ++it)
      {
        *it = EEPROM.read(addr++);
      }
      return 1;
    }

    void save()
    {
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
    }

    void reset()
    {
      config = ModelConfig();
    }

    static const uint8_t EEPROM_MAGIC   = 0xA5;
    static const uint8_t EEPROM_VERSION = 0x01;
};

}

#endif
