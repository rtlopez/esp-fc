#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"

namespace Espfc {

enum GyroRate {
  GYRO_RATE_500 = 0x00,
  GYRO_RATE_333 = 0x01,
  GYRO_RATE_250 = 0x02,
  GYRO_RATE_200 = 0x03,
  GYRO_RATE_166 = 0x04,
  GYRO_RATE_100 = 0x05,
  GYRO_RATE_50  = 0x06
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

enum ModelFrame {
  FRAME_QUAD_X = 0x01
};

// working data
struct ModelState
{
  unsigned long timestamp;

  VectorInt16 gyroRaw;
  VectorInt16 accelRaw;
  VectorInt16 magRaw;

  VectorFloat gyro;
  VectorFloat accel;
  VectorFloat mag;
  VectorFloat magAccel;
  VectorFloat pose;

  Quaternion accelQ;

  VectorFloat rate;
  VectorFloat angle;

  float altitude;
  float altitudeVelocity;

  PidState pid[3];
  float input[8];
  float output[3];

  long channel[4];

  // other state
  Kalman kalman[3];
  VectorFloat accelPrev;

  float accelScale;
  float gyroScale;
  VectorFloat gyroBias;
  float gyroBiasAlpha;
  float gyroBiasSamples;
  long gyroBiasValid;

  long gyroSampleRate;
  long gyroSampleInterval;

  long magSampleInterval;
  long magSampleRate;
  float magScale;
  unsigned long magTimestamp;

  unsigned long telemetryTimestamp;
};

// persistent data
struct ModelConfig
{
  long gyroFifo;
  long gyroDlpf;
  long gyroFsr;
  long accelFsr;
  long gyroSampleRate;

  long magSampleRate;
  long magFsr;
  long magAvr;

  float gyroFilterAlpha;
  float accelFilterAlpha;
  float magFilterAlpha;

  long modelFrame;

  long inputMin[8];
  long inputNeutral[8];
  long inputMax[8];

  long channelMin[4];
  long channelNeutral[4];
  long channelMax[4];

  Pid pid[3];

  long telemetry;
  long telemetryInterval;
};

class Model
{
  public:
    Model() {
      config.gyroFifo = true;
      config.gyroDlpf = GYRO_DLPF_256;
      config.gyroFsr  = GYRO_FS_2000;
      config.accelFsr = ACCEL_FS_8;
      config.gyroSampleRate = GYRO_RATE_500;
      config.magSampleRate = MAG_RATE_75;
      config.magAvr = MAG_AVERAGING_1;

      config.accelFilterAlpha = 1.f;
      config.gyroFilterAlpha = 1.f;
      config.magFilterAlpha = 1.f;

      config.telemetry = true;
      config.telemetryInterval = 200;
    }
    union {
      ModelState state;
      char * stateAddr;
    };
    union {
      ModelConfig config;
      char * configAddr;
    };
};

}

#endif
