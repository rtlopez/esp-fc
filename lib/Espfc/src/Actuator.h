#ifndef _ESPFC_ACTUATOR_H_
#define _ESPFC_ACTUATOR_H_

#include "Model.h"
#include "MathUtil.h"

namespace Espfc {

class Actuator
{
  public:
    Actuator(Model& model): _model(model) {}

    int begin()
    {
      _model.state.modeMask = 0;
      _model.state.airmodeAllowed = false;
      return 1;
    }

    int update()
    {
      if(!_model.state.actuatorTimer.check()) return 0;

      Stats::Measure(_model.state.stats, COUNTER_ACTUATOR);
      updateArming();
      updateModeMask();
      updateAirMode();
      updateScaler();
      updateBuzzer();
      return 1;
    }

  private:
    void updateScaler()
    {
      for(size_t i = 0; i < SCALER_COUNT; i++)
      {
        uint32_t mode = _model.config.scaler[i].dimension;
        if(!mode) continue;

        short c = _model.config.scaler[i].channel;
        if(c < AXIS_AUX_1) continue;

        float v = _model.state.input[c];
        float min = _model.config.scaler[i].minScale * 0.01f;
        float max = _model.config.scaler[i].maxScale * 0.01f;
        float scale = Math::map3(v, -1.f, 0.f, 1.f, min, min < 0 ? 0.f : 1.f, max);
        for(size_t x = 0; x < AXES; x++)
        {
          if(
            (x == AXIS_ROLL   && (mode & ACT_AXIS_ROLL))  ||
            (x == AXIS_PITCH  && (mode & ACT_AXIS_PITCH)) ||
            (x == AXIS_YAW    && (mode & ACT_AXIS_YAW))   ||
            (x == AXIS_THRUST && (mode & ACT_AXIS_THRUST))
          )
          {

            if(mode & ACT_INNER_P) _model.state.innerPid[x].pScale = scale;
            if(mode & ACT_INNER_I) _model.state.innerPid[x].iScale = scale;
            if(mode & ACT_INNER_D) _model.state.innerPid[x].dScale = scale;
            if(mode & ACT_INNER_F) _model.state.innerPid[x].fScale = scale;

            if(mode & ACT_OUTER_P) _model.state.outerPid[x].pScale = scale;
            if(mode & ACT_OUTER_I) _model.state.outerPid[x].iScale = scale;
            if(mode & ACT_OUTER_D) _model.state.outerPid[x].dScale = scale;
            if(mode & ACT_OUTER_F) _model.state.outerPid[x].fScale = scale;

          }
        }
      }
    }

    void updateArming()
    {
      int errors = _model.state.i2cErrorDelta;
      int flags = 0;
      if(!_model.state.gyroPresent || errors)
      {
        flags |= ARMING_DISABLED_NO_GYRO;
      }
      if(!_model.isThrottleLow())
      {
        flags |= ARMING_DISABLED_THROTTLE;
      }
      if(_model.calibrationActive())
      {
        flags |= ARMING_DISABLED_CALIBRATING;
      }
      if(!_model.state.inputLinkValid)
      {
        flags |= ARMING_DISABLED_RX_FAILSAFE;
      }
      _model.state.armingDisabledFlags = (ArmingDisabledFlags)flags;
      _model.state.i2cErrorDelta = 0;
    }

    void updateModeMask()
    {
      uint32_t newMask = 0;
      for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        ActuatorCondition * c = &_model.config.conditions[i];
        if(!(c->min < c->max)) continue; // inactive

        int16_t min = c->min; // * 25 + 900;
        int16_t max = c->max; // * 25 + 900;
        size_t ch = c->ch;    // + AXIS_AUX_1;
        if(ch < AXIS_AUX_1 || ch >= AXIS_COUNT) continue; // invalid channel

        int16_t val = _model.state.inputUs[ch];
        if(val > min && val < max)
        {
          newMask |= 1 << c->id;
        }
      }

      _model.state.modeMaskNew = newMask;

      for(size_t i = 0; i < MODE_COUNT; i++)
      {
        bool next = newMask & (1 << i);
        bool prev = _model.state.modeMaskPrev & (1 << i);
        if(next != prev && !canChangeMode((FlightMode)i, next))
        {
          if(next) newMask &= ~(1 << i); // block activation, clear bit
          else newMask |= (1 << i); // keep previous, set bit
        }
      }

      _model.state.modeMaskPrev = _model.state.modeMask;
      _model.state.modeMask = newMask;
    }

    bool canChangeMode(FlightMode mode, bool val)
    {
      switch(mode)
      {
        case MODE_ARMED:
          return (val && !_model.armingDisabled()) || (!val && _model.isThrottleLow());
        case MODE_ANGLE:
          return _model.accelActive();
        case MODE_AIRMODE:
          return (val && _model.state.airmodeAllowed) || !val;
        default:
          return true;
      }
    }

    void updateAirMode()
    {
      bool armed = _model.isActive(MODE_ARMED);
      if(!armed)
      {
        _model.state.airmodeAllowed = false;
      }
      if(armed && !_model.state.airmodeAllowed && _model.state.inputUs[AXIS_THRUST] > 1400) // activate airmode in the air
      {
        _model.state.airmodeAllowed = true;
      }
    }

    void updateBuzzer()
    {
      if(_model.isActive(MODE_FAILSAFE))
      {
        _model.state.buzzer.play(BEEPER_RX_LOST);
      }
      if(_model.state.battery.warn(_model.config.vbatCellWarning))
      {
        _model.state.buzzer.play(BEEPER_BAT_LOW);
      }
      if(_model.isActive(MODE_BUZZER))
      {
        _model.state.buzzer.play(BEEPER_RX_SET);
      }
      if((_model.hasChanged(MODE_ARMED)))
      {
        _model.state.buzzer.push(_model.isActive(MODE_ARMED) ? BEEPER_ARMING : BEEPER_DISARMING);
      }
    }

    Model& _model;
};

}

#endif
