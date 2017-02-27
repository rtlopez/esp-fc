#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"
#include "EEPROM.h"

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

enum GyroFsr {
  GYRO_FS_250  = 0x00,
  GYRO_FS_500  = 0x01,
  GYRO_FS_1000 = 0x02,
  GYRO_FS_2000 = 0x03
};

enum AccelFsr {
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
  FUSION_MADGWICK      = 0x00,
  FUSION_RTQF          = 0x01,
  FUSION_LERP          = 0x02,
  FUSION_COMPLEMENTARY = 0x03,
  FUSION_KALMAN        = 0x04
};

enum FlightMode {
  MODE_OFF      = 0x00,
  MODE_DIRECT   = 0x01,
  MODE_RATE     = 0x02,
  MODE_ANGLE    = 0x03
};

enum ModelFrame {
  FRAME_UNCONFIGURED  = 0x00,
  FRAME_QUAD_X        = 0x01,
  FRAME_BALANCE_ROBOT = 0x02,
  FRAME_GIMBAL        = 0x03
};

enum ActuatorConfig {
  ACT_INNER_P   = 1 << 0,
  ACT_INNER_I   = 1 << 1,
  ACT_INNER_D   = 1 << 2,
  ACT_OUTER_P   = 1 << 3,
  ACT_OUTER_I   = 1 << 4,
  ACT_OUTER_D   = 1 << 5,
  ACT_AXIS_X    = 1 << 6,
  ACT_AXIS_Y    = 1 << 7,
  ACT_AXIS_Z    = 1 << 8
};

const size_t OUTPUT_CHANNELS = 4;
const size_t INPUT_CHANNELS  = 8;

const size_t AXES        = 4;
const size_t AXIS_ROLL   = 0;  // x
const size_t AXIS_PITH   = 1;  // y
const size_t AXIS_YAW    = 2;  // z
const size_t AXIS_THRUST = 3;  // throttle channel index

// working data
struct ModelState
{
  unsigned long timestamp;

  VectorInt16 gyroRaw;
  VectorInt16 accelRaw;
  VectorInt16 magRaw;

  VectorFloat gyro;
  VectorFloat gyroPose;
  Quaternion gyroPoseQ;

  VectorFloat accel;
  VectorFloat accelPose;
  VectorFloat accelPose2;
  Quaternion accelPoseQ;

  VectorFloat mag;
  VectorFloat magPose;

  VectorFloat pose;
  Quaternion poseQ;

  VectorFloat rate;
  VectorFloat angle;
  Quaternion angleQ;

  VectorFloat desiredAngle;
  Quaternion desiredAngleQ;

  VectorFloat desiredRotation;
  Quaternion desiredRotationQ;

  Quaternion boardRotationQ;

  short flightMode;
  bool armed;

  float altitude;
  float altitudeVelocity;

  PidState innerPid[AXES];
  PidState outerPid[AXES];

  short inputUs[INPUT_CHANNELS];
  float input[INPUT_CHANNELS];

  float rateDesired[AXES];
  float angleDesired[AXES];

  float rateMax[AXES];
  float angleMax[AXES];

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

  long gyroSampleRate;
  long gyroSampleInterval;
  float gyroSampleIntervalFloat;

  long magSampleInterval;
  long magSampleRate;
  float magScale;

  float magCalibrationData[3][2];
  bool magCalibrationValid;

  unsigned long magTimestamp;
  unsigned long controllerTimestamp;
  unsigned long telemetryTimestamp;
  unsigned long mixerTimestamp;

  bool newGyroData;
  bool newInputData;
};

// persistent data
struct ModelConfig
{
  short ppmPin;
  bool gyroFifo;
  long gyroDlpf;
  long gyroFsr;
  long accelFsr;
  short gyroSampleRate;

  short magSampleRate;
  short magFsr;
  short magAvr;

  float gyroFilterAlpha;
  float accelFilterAlpha;
  float magFilterAlpha;

  short magCalibration;
  short magEnable;

  VectorFloat magCalibrationScale;
  VectorFloat magCalibrationOffset;
  VectorFloat boardRotation;

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

  short outputMin[OUTPUT_CHANNELS];
  short outputNeutral[OUTPUT_CHANNELS];
  short outputMax[OUTPUT_CHANNELS];

  short outputPin[OUTPUT_CHANNELS];
  short pwmRate;

  Pid innerPid[AXES];
  Pid outerPid[AXES];
  float rateMax[AXES];
  float angleMax[AXES];

  bool telemetry;
  short telemetryInterval;

  short fusionMode;
};

class Model
{
  public:
    Model() {
      config.gyroFifo = 1;
      config.gyroDlpf = GYRO_DLPF_188;
      config.gyroFsr  = GYRO_FS_2000;
      config.accelFsr = ACCEL_FS_8;
      config.gyroSampleRate = GYRO_RATE_333;
      config.magSampleRate = MAG_RATE_75;
      config.magAvr = MAG_AVERAGING_1;
      config.magCalibration = 0;
      config.magEnable = 0;

      config.accelFilterAlpha = 1;
      config.gyroFilterAlpha = 1;
      config.magFilterAlpha = 1;
      config.fusionMode = FUSION_MADGWICK;

      for(size_t i = 0; i < 3; i++)
      {
        config.magCalibrationOffset.set(i, 0.f);
        config.magCalibrationScale.set(i, 1.f);
      }

      config.telemetry = 1;
      config.telemetryInterval = 40;

      // output config
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        config.outputMin[i] = 1000;
        config.outputNeutral[i] = 1050;
        config.outputMax[i] = 2000;
      }
      config.outputPin[0] = D5;
      config.outputPin[1] = D6;
      config.outputPin[2] = D7;
      config.outputPin[3] = D8;
      config.modelFrame = FRAME_QUAD_X;
      config.pwmRate = 100;

      // input config
      config.ppmPin = D4; // GPIO2
      config.inputMap[0] = 0; // roll
      config.inputMap[1] = 1; // pitch
      config.inputMap[2] = 3; // yaw
      config.inputMap[3] = 2; // throttle
      config.inputMap[4] = 4; // flight mode
      config.inputMap[5] = 5; // free
      config.inputMap[6] = 6; // free
      config.inputMap[7] = 7; // free

      config.flightModeChannel = 4;
      config.flightModes[0] = MODE_ANGLE;
      config.flightModes[1] = MODE_RATE;
      config.flightModes[2] = MODE_DIRECT;

      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        config.inputMin[i] = 1000;
        config.inputNeutral[i] = 1500;
        config.inputMax[i] = 2000;
      }
      config.inputNeutral[AXIS_THRUST] = 1050; // override for thrust

      // controller config
      for(size_t i = 0; i < AXES; i++)
      {
        state.kalman[i] = Kalman();
        state.outerPid[i] = PidState();
        state.innerPid[i] = PidState();
        config.outerPid[i] = Pid(1, 0, 0, 0.3);
        config.innerPid[i] = Pid(1, 0, 0, 0.3);
      }

      config.rateMax[AXIS_PITH] = config.rateMax[AXIS_ROLL] = 200; // deg/s
      config.angleMax[AXIS_PITH] = config.angleMax[AXIS_ROLL] = 30; // deg
      config.rateMax[AXIS_YAW] = 200; // deg/s
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
