#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"
#include "Kalman.h"

namespace Espfc {

enum GyroDlpf {
  DLPF_256 = 0x00,
  DLPF_188 = 0x01,
  DLPF_98  = 0x02,
  DLPF_42  = 0x03,
  DLPF_20  = 0x04,
  DLPF_10  = 0x05,
  DLPF_5   = 0x06
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
};

struct ModelConfig
{
  int gyroFifo;
  int gyroDlpf;
  int gyroFsr;
  int accelFsr;
  int gyroSampleRate;
  int magSampleRate;

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
