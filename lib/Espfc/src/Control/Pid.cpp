#include "Pid.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"
#include <algorithm>

namespace Espfc {

namespace Control {

Pid::Pid():
  rate(1.0f), dt(1.0f), Kp(0.1), Ki(0.f), Kd(0.f), Kf(0.0f),
  iLimitLow(-0.3f), iLimitHigh(0.3f), iReset(0.0f), oLimitLow(-1.f), oLimitHigh(1.f),
  pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f),
  error(0.f), iTermError(0.f),
  pTerm(0.f), iTerm(0.f), dTerm(0.f), fTerm(0.f),
  prevMeasurement(0.f), prevError(0.f), prevSetpoint(0.f),
  ftermDerivative(true), outputSaturated(false),
  itermRelax(ITERM_RELAX_OFF), itermRelaxFactor(1.0f), itermRelaxBase(0.f)
  {}

void Pid::begin()
{
  dt = 1.f / rate;
}

void Pid::resetIterm()
{
  iTerm = iReset;
}

float FAST_CODE_ATTR Pid::update(float setpoint, float measurement)
{
  error = setpoint - measurement;
  
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
        itermRelaxFactor = std::max(0.0f, 1.0f - std::abs(Utils::toDeg(itermRelaxBase)) * 0.025f); // (itermRelaxBase / 40)
        if(!incrementOnly || increasing) iTermError *= itermRelaxFactor;
      }
      iTerm += Ki * iScale * iTermError * dt;
      iTerm = std::clamp(iTerm, iLimitLow, iLimitHigh);
    }
  }
  else
  {
    iTerm = 0; // zero integral
  }

  // D-term
  if(Kd > 0.f && dScale > 0.f)
  {
    //dTerm = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasurement - measure) * (1.f - dGamma)) / dt);
    dTerm = Kd * dScale * ((prevMeasurement - measurement) * rate);
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
    if(ftermDerivative)
    {
      fTerm = Kf * fScale * (setpoint - prevSetpoint) * rate;
    }
    else
    {
      fTerm = Kf * fScale * setpoint;
    }
    fTerm = ftermFilter.update(fTerm);
  }
  else
  {
    fTerm = 0;
  }

  prevMeasurement = measurement;
  prevError = error;
  prevSetpoint = setpoint;

  return std::clamp(pTerm + iTerm + dTerm + fTerm, oLimitLow, oLimitHigh);
}

}

}
