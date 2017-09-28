#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "EEPROM.h"
#include "Filter.h"

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
  SERIAL_SPEED_250000 = 250000
};

enum SerialPort {
  SERIAL_UART_NONE = 0,
  SERIAL_UART_1,
  SERIAL_UART_2
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
  float gyroThrustScale;

  VectorFloat controlAngle;
  VectorFloat desiredAngle;
  Quaternion desiredAngleQ;

  float desiredRate[AXES];

  Quaternion boardRotationQ;

  short flightMode;
  bool armed;

  float altitude;

  PidState innerPid[AXES];
  PidState outerPid[AXES];

  short inputUs[INPUT_CHANNELS];
  float input[INPUT_CHANNELS];
  unsigned long inputDelay;

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

  float magCalibrationData[3][2];
  bool magCalibrationValid;

  uint32_t telemetryTimestamp;
  bool telemetryUpdate;
};

// persistent data
struct ModelConfig
{
  uint8_t ppmPin;
  uint8_t ppmMode;

  bool gyroFifo;
  GyroDlpf gyroDlpf;
  GyroGain gyroFsr;
  AccelGain accelFsr;
  GyroRate gyroSampleRate;

  MagRate magSampleRate;
  MagGain magFsr;
  MagAvg magAvr;

  float velocityFilterAlpha;
  float gyroDeadband;

  bool magCalibration;
  bool magEnable;

  VectorFloat magCalibrationScale;
  VectorFloat magCalibrationOffset;
  VectorFloat boardRotation;

  FilterType gyroFilterType;
  uint16_t gyroFilterCutFreq;
  FilterType accelFilterType;
  uint16_t accelFilterCutFreq;
  FilterType magFilterType;
  uint16_t magFilterCutFreq;
  FilterType dtermFilterType;
  uint16_t dtermFilterCutFreq;

  short modelFrame;
  short flightModeChannel;
  short flightModes[3];

  long actuatorConfig[3];
  short actuatorChannels[3];
  float actuatorMin[3];
  float actuatorMax[3];

  short inputMin[INPUT_CHANNELS];
  short inputNeutral[INPUT_CHANNELS];
  short inputMax[INPUT_CHANNELS];
  short inputMap[INPUT_CHANNELS];
  uint8_t inputDeadband;
  float inputAlpha;
  uint8_t inputExpo[3];
  uint8_t inputRate[3];
  uint8_t inputSuperRate[3];

  short outputMin[OUTPUT_CHANNELS];
  short outputNeutral[OUTPUT_CHANNELS];
  short outputMax[OUTPUT_CHANNELS];

  short outputPin[OUTPUT_CHANNELS];
  short pwmRate;

  Pid innerPid[AXES];
  Pid outerPid[AXES];
  float angleMax[AXES];
  float velocityMax[AXES];
  short loopSync;
  short mixerSync;

  float lowThrottleTreshold;
  bool lowThrottleZeroIterm;
  bool lowThrottleMotorStop;

  bool telemetry;
  uint32_t telemetryInterval;
  SerialPort telemetryPort;

  bool blackbox;
  SerialPort blackboxPort;
  SerialPort cliPort;

  SerialSpeed uart1Speed;
  SerialSpeed uart2Speed;

  short fusionMode;
};

class Model
{
  public:
    Model() {
      config.gyroFifo = 0;
      config.gyroDlpf = GYRO_DLPF_256;
      config.gyroFsr  = GYRO_FS_2000;
      config.accelFsr = ACCEL_FS_8;
      config.gyroSampleRate = GYRO_RATE_500;

      config.magEnable = 0;
      config.magSampleRate = MAG_RATE_75;
      config.magAvr = MAG_AVERAGING_1;
      config.magCalibration = 0;

      config.loopSync = 1;
      config.mixerSync = 1;

      //config.gyroDeadband = radians(0.1); // deg/s

      config.velocityFilterAlpha = 0.1f;
      //config.fusionMode = FUSION_EXPERIMENTAL;
      config.fusionMode = FUSION_MADGWICK;

      config.gyroFilterType = FILTER_PT1;
      config.gyroFilterCutFreq = 100;
      config.accelFilterType = FILTER_PT1;
      config.accelFilterCutFreq = 20;
      config.magFilterType = FILTER_PT1;
      config.magFilterCutFreq = 20;
      config.dtermFilterType = FILTER_BIQUAD;
      config.dtermFilterCutFreq = 100;

      for(size_t i = 0; i < 3; i++)
      {
        config.magCalibrationOffset.set(i, 0.f);
        config.magCalibrationScale.set(i, 1.f);
      }

      config.uart1Speed = SERIAL_SPEED_115200;

      //config.uart2Speed = SERIAL_SPEED_115200;
      //config.uart2Speed = SERIAL_SPEED_230400;
      config.uart2Speed = SERIAL_SPEED_250000;
      //config.uart2Speed = SERIAL_SPEED_NONE;

      config.cliPort = SERIAL_UART_1;

      config.telemetry = 0;
      config.telemetryInterval = 500 * 1000;
      config.telemetryPort = SERIAL_UART_1;

      config.blackbox = 1;
      config.blackboxPort = SERIAL_UART_2;
      //config.blackboxPort = SERIAL_UART_NONE;

      // output config
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        config.outputMin[i] = 1050;
        config.outputNeutral[i] = 1500;
        config.outputMax[i] = 1950;
      }

      config.outputPin[0] = D3; // D8; // -1 off
      config.outputPin[1] = D5; // D6;
      config.outputPin[2] = D6; // D5;
      config.outputPin[3] = D8; // D3;
      config.modelFrame = FRAME_QUAD_X;
      //config.modelFrame = FRAME_DIRECT;
      config.pwmRate = 125;    // 125 max

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
        config.inputRate[i] = 110;
        config.inputExpo[i] = 0;
        config.inputSuperRate[i] = 40;
      }
      config.inputRate[AXIS_YAW] = 110;
      config.inputExpo[AXIS_YAW] = 0;
      config.inputSuperRate[AXIS_YAW] = 20;

      config.flightModeChannel = 4;
      config.flightModes[0] = MODE_OFF;
      config.flightModes[1] = MODE_RATE;
      config.flightModes[2] = MODE_ANGLE;

      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        config.inputMin[i] = 1000;
        config.inputNeutral[i] = 1500;
        config.inputMax[i] = 2000;
      }
      config.inputMin[AXIS_YAW] = 2000;        // invert Yaw axis
      config.inputMax[AXIS_YAW] = 1000;

      config.inputDeadband = 3; // us
      config.inputAlpha = 0.8f;

      // controller config
      for(size_t i = 0; i < AXES; i++)
      {
        state.kalman[i] = Kalman();
        state.outerPid[i] = PidState();
        state.innerPid[i] = PidState();
        config.outerPid[i] = Pid(3.00f, 0.00f, 0.0000f, 0.0f, 0.0f, radians(300));
        config.innerPid[i] = Pid(0.10f, 0.40f, 0.0010f, 0.3f, 1.0f);
      }

      config.innerPid[AXIS_ROLL].Kp *= 0.8;
      config.innerPid[AXIS_ROLL].Ki *= 0.8;
      config.innerPid[AXIS_ROLL].Kd *= 0.8;

      config.innerPid[AXIS_YAW].Kp = 0.2;
      config.innerPid[AXIS_YAW].Ki = 0.4;
      config.innerPid[AXIS_YAW].Kd = 0.0002f;

      config.angleMax[AXIS_PITCH] = config.angleMax[AXIS_ROLL] = radians(50);  // deg
      config.velocityMax[AXIS_PITCH] = config.velocityMax[AXIS_ROLL] = 0.5; // m/s

      config.lowThrottleTreshold = -0.9f;
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

      state.gyroThrustScale = 0.02;
    }

    union {
      ModelState state;
      char * stateAddr;
    };
    union {
      ModelConfig config;
      char * configAddr;
    };

    int load()
    {
      int addr = 0;
      uint8_t magic = EEPROM.read(addr++);
      if(EEPROM_MAGIC != magic) return -1;

      uint8_t version = EEPROM.read(addr++);

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config.magCalibrationScale);
      uint8_t * end = begin + sizeof(VectorFloat) * 2;
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

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config.magCalibrationScale);
      uint8_t * end = begin + sizeof(VectorFloat) * 2;
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
