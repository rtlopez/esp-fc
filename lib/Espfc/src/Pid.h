#ifndef _ESPFC_PID_H_
#define _ESPFC_PID_H_

#include "Math.h"
#include "Filter.h"

namespace Espfc {

struct PidState
{
  public:
    PidState(): pTerm(0), iTerm(0), dTerm(0), prevInput(0), pScale(1.f), iScale(1.f), dScale(1.f) {}
    float error;
    float pTerm;
    float iTerm;
    float dTerm;
    float prevInput;
    float prevError;
    float pScale;
    float iScale;
    float dScale;
    Filter dtermFilter;
};

class Pid
{
  public:
    Pid(float kp = 1, float ki = 0, float kd = 0, float il = 0.3, float da = 1, float dg = 0):
      Kp(kp), Ki(ki), Kd(kd), iLimit(il), dAlpha(da), dGamma(dg) {}

    float update(float setpoint, float input, float dt, PidState& state)
    {
      float error = state.error = setpoint - input;
      state.pTerm = Kp * error * state.pScale;
      if(state.iScale > 0.01)
      {
        state.iTerm += Ki * error * dt * state.iScale;
        state.iTerm = Math::bound(state.iTerm, -iLimit, iLimit);
      }
      else
      {
        state.iTerm = 0; // zero integral
      }

      float dTerm = 0;
      if(Kd > 0 && dt > 0)
      {
        //dTerm = (Kd * (error - state.prevError) / dt) * state.dScale;
        // OR
        //dTerm = (Kd * (state.prevInput - input) / dt) * state.dScale;
        // OR BOTH
        dTerm = (Kd * (((error - state.prevError) * dGamma) + (state.prevInput - input) * (1.f - dGamma)) / dt) * state.dScale;
      }
      //state.dTerm = (1.f - dAlpha) * state.dTerm + dAlpha * dTerm;
      state.dTerm = state.dtermFilter.update(dTerm);

      float output = Math::bound(state.pTerm + state.iTerm + state.dTerm, -1.f, 1.f);

      state.prevInput = input;
      state.prevError = state.error;

      return output;
    }

    float Kp;
    float Ki;
    float Kd;
    float iLimit;
    float dAlpha;
    float dGamma;
};

}

#endif
