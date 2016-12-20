#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"

namespace Espfc {

enum GyroRate {
  GYRO_RATE_50  = 50,
  GYRO_RATE_100 = 100,
  GYRO_RATE_150 = 150,
  GYRO_RATE_200 = 200,
  GYRO_RATE_250 = 250,
  GYRO_RATE_333 = 333,
  GYRO_RATE_500 = 500
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
  MAG_RATE_3    = 0x02,
  MAG_RATE_7P5  = 0x03,
  MAG_RATE_15   = 0x04,
  MAG_RATE_30   = 0x05,
  MAG_RATE_75   = 0x06,
};

enum MagAvg {
  MAG_AVERAGING_1 = 0x00,
  MAG_AVERAGING_2 = 0x01,
  MAG_AVERAGING_4 = 0x02,
  MAG_AVERAGING_8 = 0x03
};

struct ModelState
{
  uint32_t timestamp;

  VectorInt16 gyroRaw;
  VectorInt16 accelRaw;
  VectorInt16 magRaw;

  VectorFloat gyro;
  VectorFloat accel;
  VectorFloat mag;
  VectorFloat pose;

  VectorFloat rate;
  VectorFloat angle;

  float altitude;
  float altitudeVelocity;

  PidState pid[3];
  float input[8];
  float output[3];

  int channel[4];
  Kalman kalman[3];

  VectorFloat accelPrev;

  float gyroScale;
  float accelScale;

  VectorFloat gyroBias;
  float gyroBiasAlpha;
  float gyroBiasSamples;
  bool gyroBiasValid;

  int gyroSampleInterval;

  int magSampleInterval;
  int magSampleRate;
  uint32_t magTimestamp;
};

struct ModelConfig
{
  int gyroFifo;
  int gyroDlpf;
  int gyroFsr;
  int accelFsr;
  int gyroSampleRate;

  int magSampleRate;
  int magFsr;
  int magAvr;

  int inputMin[8];
  int inputNeutral[8];
  int inputMax[8];

  int channelMin[4];
  int channelNeutral[4];
  int channelMax[4];

  Pid pid[3];

  int telemetry;
  int telemetryInterval;
  uint32_t telemetryTimestamp;
};

class Model
{
  public:
    Model() {}
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
