#ifndef _ESPFC_CONTROL_PID_H_
#define _ESPFC_CONTROL_PID_H_

#include "Math/Utils.h"
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
    Pid():
      rate(1.0f), dt(1.0f), Kp(0.1), Ki(0.f), Kd(0.f), Kf(0.0f), iLimit(0.3f), oLimit(1.f),
      pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f),
      error(0.f), iTermError(0.f),
      pTerm(0.f), iTerm(0.f), dTerm(0.f), fTerm(0.f),
      prevMeasure(0.f), prevError(0.f), prevSetpoint(0.f),
      outputSaturated(false),
      itermRelax(ITERM_RELAX_OFF), itermRelaxBase(0.f), itermRelaxFactor(1.0f)
      {}

    void begin()
    {
      dt = 1.f / rate;
    }

    float update(float setpoint, float measure)
    {
      error = setpoint - measure;
      
      // P-term
      pTerm = Kp * error * pScale;
      pTerm = ptermFilter.update(pTerm);

      // I-term
      iTermError = error;
      if(Ki > 0.f && iScale > 0.f)
      {
        if(!outputSaturated)
        {
          // I-term relax
          if(itermRelax)
          {
            const bool increasing = (iTerm > 0 && iTermError > 0) || (iTerm < 0 && iTermError < 0);
            const bool incrementOnly = itermRelax == ITERM_RELAX_RP_INC || itermRelax == ITERM_RELAX_RPY_INC;
            itermRelaxBase = setpoint - itermRelaxFilter.update(setpoint);
            itermRelaxFactor = std::max(0.0f, 1.0f - abs(Math::toDeg(itermRelaxBase)) * 0.025f); // (itermRelaxBase / 40)
            if(!incrementOnly || increasing) iTermError *= itermRelaxFactor;
          }
          iTerm += Ki * iScale * iTermError * dt;
          iTerm = Math::clamp(iTerm, -iLimit, iLimit);
        }
      }
      else
      {
        iTerm = 0; // zero integral
      }

      // D-term
      if(Kd > 0.f && dScale > 0.f)
      {
        //dTerm = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasure - measure) * (1.f - dGamma)) / dt);
        dTerm = Kd * dScale * ((prevMeasure - measure) * rate);
        dTerm = dtermNotchFilter.update(dTerm);
        dTerm = dtermFilter.update(dTerm);
        dTerm = dtermFilter2.update(dTerm);
      }
      else
      {
        dTerm = 0;
      }

      // F-term
      if(Kf > 0.f && fScale > 0.f)
      {
        fTerm = Kf * fScale * (setpoint - prevSetpoint) * rate;
        fTerm = ftermFilter.update(fTerm);
      }
      else
      {
        fTerm = 0;
      }

      prevMeasure = measure;
      prevError = error;
      prevSetpoint = setpoint;

      return Math::clamp(pTerm + iTerm + dTerm + fTerm, -oLimit, oLimit);
    }

    float rate;
    float dt;

    float Kp;
    float Ki;
    float Kd;
    float Kf;

    float iLimit;
    float oLimit;
    //float dGamma;

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

    float prevMeasure;
    float prevError;
    float prevSetpoint;

    bool outputSaturated;
    int8_t itermRelax;
    float itermRelaxFactor;
    float itermRelaxBase;
};

}

}

#endif
