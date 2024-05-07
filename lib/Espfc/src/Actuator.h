#ifndef _ESPFC_ACTUATOR_H_
#define _ESPFC_ACTUATOR_H_

#include "Model.h"
#include "Math/Utils.h"

namespace Espfc {

class Actuator
{
  public:
    Actuator(Model& model): _model(model) {}

    int begin()
    {
      _model.state.modeMask = 0;
      _model.state.modeMaskPrev = 0;
      _model.state.modeMaskPresent = 0;
      _model.state.modeMaskSwitch = 0;
      for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        const auto &c = _model.config.conditions[i];
        if(!(c.min < c.max)) continue; // inactive
        if(c.ch < AXIS_AUX_1 || c.ch >= AXIS_COUNT) continue; // invalid channel
        _model.state.modeMaskPresent |= 1 << c.id;
      }
      _model.state.airmodeAllowed = false;
      _model.state.rescueConfigMode = RESCUE_CONFIG_PENDING;
      return 1;
    }

    int update()
    {
      uint32_t startTime = micros();
      Stats::Measure(_model.state.stats, COUNTER_ACTUATOR);
      updateArmingDisabled();
      updateModeMask();
      updateArmed();
      updateAirMode();
      updateScaler();
      updateBuzzer();
      updateDynLpf();
      updateRescueConfig();

      if(_model.config.debugMode == DEBUG_PIDLOOP)
      {
        _model.state.debug[4] = micros() - startTime;
      }

      return 1;
    }

  #ifndef UNIT_TEST
  private:
  #endif

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

    void updateArmingDisabled()
    {
      int errors = _model.state.i2cErrorDelta;
      _model.state.i2cErrorDelta = 0;

      _model.setArmingDisabled(ARMING_DISABLED_NO_GYRO,       !_model.state.gyroPresent || errors);
      _model.setArmingDisabled(ARMING_DISABLED_FAILSAFE,       _model.state.failsafe.phase != FC_FAILSAFE_IDLE);
      _model.setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE,    _model.state.inputRxLoss || _model.state.inputRxFailSafe);
      _model.setArmingDisabled(ARMING_DISABLED_THROTTLE,      !_model.isThrottleLow());
      _model.setArmingDisabled(ARMING_DISABLED_CALIBRATING,    _model.calibrationActive());
      _model.setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL, _model.config.output.protocol == ESC_PROTOCOL_DISABLED);
      _model.setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED, _model.state.rescueConfigMode == RESCUE_CONFIG_ACTIVE);
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

      _model.updateSwitchActive(newMask);

      _model.setArmingDisabled(ARMING_DISABLED_FAILSAFE,    _model.state.failsafe.phase != FC_FAILSAFE_IDLE);
      _model.setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE, _model.isSwitchActive(MODE_FAILSAFE));
      _model.setArmingDisabled(ARMING_DISABLED_ARM_SWITCH,  _model.armingDisabled() && _model.isSwitchActive(MODE_ARMED));

      if(_model.state.failsafe.phase != FC_FAILSAFE_IDLE)
      {
        newMask |= (1 << MODE_FAILSAFE);
      }

      for(size_t i = 0; i < MODE_COUNT; i++)
      {
        bool newVal = newMask & (1 << i);
        bool oldVal = _model.state.modeMask & (1 << i);
        if(newVal == oldVal) continue; // mode unchanged
        if(newVal && !canActivateMode((FlightMode)i))
        {
          newMask &= ~(1 << i); // block activation, clear bit
        }
      }

      _model.updateModes(newMask);
    }

    bool canActivateMode(FlightMode mode)
    {
      switch(mode)
      {
        case MODE_ARMED:
          return !_model.armingDisabled() && _model.isThrottleLow();
        case MODE_ANGLE:
          return _model.accelActive();
        case MODE_AIRMODE:
          return _model.state.airmodeAllowed;
        default:
          return true;
      }
    }

    void updateArmed()
    {
      if((_model.hasChanged(MODE_ARMED)))
      {
        bool armed = _model.isModeActive(MODE_ARMED);
        if(armed)
        {
          _model.state.disarmReason = DISARM_REASON_SYSTEM;
          _model.state.rescueConfigMode = RESCUE_CONFIG_DISABLED;
        }
        else if(!armed && _model.state.disarmReason == DISARM_REASON_SYSTEM)
        {
          _model.state.disarmReason = DISARM_REASON_SWITCH;
        }
      }
    }

    void updateAirMode()
    {
      bool armed = _model.isModeActive(MODE_ARMED);
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
      if(_model.isModeActive(MODE_FAILSAFE))
      {
        _model.state.buzzer.play(BUZZER_RX_LOST);
      }
      if(_model.state.battery.warn(_model.config.vbatCellWarning))
      {
        _model.state.buzzer.play(BUZZER_BAT_LOW);
      }
      if(_model.isModeActive(MODE_BUZZER))
      {
        _model.state.buzzer.play(BUZZER_RX_SET);
      }
      if((_model.hasChanged(MODE_ARMED)))
      {
        _model.state.buzzer.push(_model.isModeActive(MODE_ARMED) ? BUZZER_ARMING : BUZZER_DISARMING);
      }
    }

    void updateDynLpf()
    {
      return; // temporary disable
      int scale = Math::clamp((int)_model.state.inputUs[AXIS_THRUST], 1000, 2000);
      if(_model.config.gyroDynLpfFilter.cutoff > 0) {
        int gyroFreq = Math::map(scale, 1000, 2000, _model.config.gyroDynLpfFilter.cutoff, _model.config.gyroDynLpfFilter.freq);
        for(size_t i = 0; i <= AXIS_YAW; i++) {
          _model.state.gyroFilter[i].reconfigure(gyroFreq);
        }
      }
      if(_model.config.dtermDynLpfFilter.cutoff > 0) {
        int dtermFreq = Math::map(scale, 1000, 2000, _model.config.dtermDynLpfFilter.cutoff, _model.config.dtermDynLpfFilter.freq);
        for(size_t i = 0; i <= AXIS_YAW; i++) {
          _model.state.innerPid[i].dtermFilter.reconfigure(dtermFreq);
        }
      }
    }

    void updateRescueConfig()
    {
      switch(_model.state.rescueConfigMode)
      {
        case RESCUE_CONFIG_PENDING:
          // if some rc frames are received, disable to prevent activate later
          if(_model.state.inputFrameCount > 100)
          {
            _model.state.rescueConfigMode = RESCUE_CONFIG_DISABLED;
          }
          if(_model.state.failsafe.phase != FC_FAILSAFE_IDLE && _model.config.rescueConfigDelay > 0 && millis() > _model.config.rescueConfigDelay * 1000)
          {
            _model.state.rescueConfigMode = RESCUE_CONFIG_ACTIVE;
          }
          break;
        case RESCUE_CONFIG_ACTIVE:
        case RESCUE_CONFIG_DISABLED:
          // nothing to do here
          break;
      }
    }

    Model& _model;
};

}

#endif
