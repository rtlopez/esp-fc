#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include "helper_3dmath.h"
#include "Pid.h"

namespace Espfc {

struct ModelState {
  VectorInt16 accelRaw;
  VectorInt16 gyroRaw;
  VectorInt16 magRaw;

  VectorFloat gyro;
  VectorFloat accel;
  VectorFloat mag;

  VectorFloat rate;
  VectorFloat pos;
  Quaternion posQ;

  float altitude;
  float altitudeVelocity;

  PidState pidState[3];
  float output[3];

  uint32_t timestamp;
};

struct ModelConfig {
  Pid pid[3];
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
