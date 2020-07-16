#ifndef _ESPFC_PID_H_
#define _ESPFC_PID_H_

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

class Pid
{
  public:
    Pid():
      Kp(0.1), Ki(0), Kd(0), iLimit(0), dGamma(0), oLimit(1.f),
      pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f),
      pTerm(0.f), iTerm(0.f), dTerm(0.f), fTerm(0.f),
      prevMeasure(0.f), prevError(0.f), prevSetpoint(0.f)
      {}

    void begin()
    {
      dt = 1.f / rate;
    }

    float update(float setpoint, float measure)
    {
      error = setpoint - measure;
      
      pTerm = Kp * error * pScale;
      pTerm = ptermFilter.update(pTerm);

      if(Ki > 0.f && iScale > 0.f)
      {
        iTerm += Ki * error * dt * iScale;
        iTerm = Math::clamp(iTerm, -iLimit, iLimit);
      }
      else
      {
        iTerm = 0; // zero integral
      }

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

      if(Kf > 0.f && fScale > 0.f)
      {
        fTerm = Kf * fScale * ((setpoint - prevSetpoint) * rate);
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

    float Kp;
    float Ki;
    float Kd;
    float Kf;

    float rate;
    float dt;

    float iLimit;
    float dGamma;
    float oLimit;

    float pScale;
    float iScale;
    float dScale;
    float fScale;

    float error;
    float pTerm;
    float iTerm;
    float dTerm;
    float fTerm;

    Filter dtermFilter;
    Filter dtermFilter2;
    Filter dtermNotchFilter;
    Filter ptermFilter;
    Filter ftermFilter;

    float prevMeasure;
    float prevError;
    float prevSetpoint;
};

}

#endif
