#pragma once

#include <cstdint>
#include "Utils/Filter.h"
#include "Utils/Math.hpp"

namespace Espfc {

// bataflight scalers
constexpr float PTERM_SCALE_BETAFLIGHT = 0.032029f;
constexpr float ITERM_SCALE_BETAFLIGHT = 0.244381f;
constexpr float DTERM_SCALE_BETAFLIGHT = 0.000529f;
constexpr float FTERM_SCALE_BETAFLIGHT = 0.00013754f;

constexpr float PTERM_SCALE = PTERM_SCALE_BETAFLIGHT * Utils::toDeg(1.0f) * 0.001f; // ~ 0.00183 = 0.032029f * 57.29 / 1000
constexpr float ITERM_SCALE = ITERM_SCALE_BETAFLIGHT * Utils::toDeg(1.0f) * 0.001f; // ~ 0.014f
constexpr float DTERM_SCALE = DTERM_SCALE_BETAFLIGHT * Utils::toDeg(1.0f) * 0.001f; // ~ 0.0000303f
constexpr float FTERM_SCALE = FTERM_SCALE_BETAFLIGHT * Utils::toDeg(1.0f) * 0.001f; // ~ 0.00000788f

constexpr float LEVEL_PTERM_SCALE = 0.1f;    // 1/10
constexpr float LEVEL_ITERM_SCALE = 0.1f;    // 1/10
constexpr float LEVEL_DTERM_SCALE = 0.001f;  // 1/1000
constexpr float LEVEL_FTERM_SCALE = 0.001f;  // 1/1000

constexpr float VEL_PTERM_SCALE = 0.02f;
constexpr float VEL_ITERM_SCALE = 0.01f;
constexpr float VEL_DTERM_SCALE = 0.001f;

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

    Utils::Filter dtermFilter;
    Utils::Filter dtermFilter2;
    Utils::Filter dtermNotchFilter;
    Utils::Filter ptermFilter;
    Utils::Filter ftermFilter;
    Utils::Filter itermRelaxFilter;

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
