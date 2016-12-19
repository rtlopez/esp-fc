#ifndef _ESPFC_PID_H_
#define _ESPFC_PID_H_

namespace Espfc {

struct PidState
{
  float pTerm;
  float iTerm;
  float dTerm;
  float prevInput;
};

class Pid
{
  public:
    Pid() {}
    float update(float setpoint, float input, float dt, PidState& state)
    {
      float error = setpoint - input;
      state.pTerm = Kp * error;
      state.iTerm += Ki * error * dt;
      state.iTerm = limit(state.iTerm, -iLimit, iLimit);

      float dTerm = 0;
      if(Kd > 0 && dt > 0)
      {
        dTerm = Kd * (state.prevInput - input) / dt;
      }
      state.dTerm = (1.f - dAlpha) * state.dTerm + dAlpha * dTerm;

      float output = limit(state.pTerm + state.iTerm + state.dTerm, -1.f, 1.f);

      state.prevInput = input;

      return output;
    }

    float limit(float val, float min, float max)
    {
      if(val < min) return min;
      if(val > max) return max;
      return val;
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
