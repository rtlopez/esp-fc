#pragma once

#include <cstdint>
#include "Filter.h"

// bataflight scalers
#define PTERM_SCALE_BETAFLIGHT 0.032029f
#define ITERM_SCALE_BETAFLIGHT 0.244381f
#define DTERM_SCALE_BETAFLIGHT 0.000529f
#define FTERM_SCALE_BETAFLIGHT 0.00013754f

#define PTERM_SCALE (PTERM_SCALE_BETAFLIGHT * RAD_TO_DEG * 0.001f) // ~ 0.00183 = 0.032029f * 57.29 / 1000
#define ITERM_SCALE (ITERM_SCALE_BETAFLIGHT * RAD_TO_DEG * 0.001f) // ~ 0.014f
#define DTERM_SCALE (DTERM_SCALE_BETAFLIGHT * RAD_TO_DEG * 0.001f) // ~ 0.0000303f
#define FTERM_SCALE (FTERM_SCALE_BETAFLIGHT * RAD_TO_DEG * 0.001f) //

#define LEVEL_PTERM_SCALE 0.1f    // 1/10
#define LEVEL_ITERM_SCALE 0.1f    // 1/10
#define LEVEL_DTERM_SCALE 0.001f  // 1/1000
#define LEVEL_FTERM_SCALE 0.001f  // 1/1000

namespace Espfc {

enum ItermRelaxType {
  ITERM_RELAX_OFF,
  ITERM_RELAX_RP,
  ITERM_RELAX_RPY,
  ITERM_RELAX_RP_INC,
  ITERM_RELAX_RPY_INC,
  ITERM_RELAX_COUNT,
};

namespace Control {

class Pid
{
  public:
    Pid();
    void begin();
    float update(float setpoint, float measure);

    float rate;
    float dt;

    float Kp;
    float Ki;
    float Kd;
    float Kf;

    float iLimit;
    float oLimit;

    float pScale;
    float iScale;
    float dScale;
    float fScale;

    float error;
    float iTermError;

    float pTerm;
    float iTerm;
    float dTerm;
    float fTerm;

    Filter dtermFilter;
    Filter dtermFilter2;
    Filter dtermNotchFilter;
    Filter ptermFilter;
    Filter ftermFilter;
    Filter itermRelaxFilter;

    float prevMeasurement;
    float prevError;
    float prevSetpoint;

    bool outputSaturated;
    int8_t itermRelax;
    float itermRelaxFactor;
    float itermRelaxBase;
};

}

}
