#ifndef _ESPFC_ESPFC_H_
#define _ESPFC_ESPFC_H_

#include "Model.h"
#include "Controller.h"
#include "Input.h"
#include "Actuator.h"
#include "Sensor.h"
#include "Fusion.h"
#include "Mixer.h"
#include "Blackbox.h"
#include "Telemetry.h"
#include "Cli.h"
#include "Hardware.h"
#include "Buzzer.h"

namespace Espfc {

class Espfc
{
  public:
    Espfc():
      _model(), _hardware(_model), _controller(_model), _input(_model), _actuator(_model), _sensor(_model),
      _mixer(_model), _blackbox(_model), _telemetry(_model), _cli(_model), _buzzer(_model)
      {}

    int begin()
    {
      PIN_DEBUG_INIT();
      _model.begin();
      _hardware.begin();
      _buzzer.begin();
      _sensor.begin();
      _input.begin();
      _actuator.begin();
      _controller.begin();
      _mixer.begin();
      _blackbox.begin();
      _telemetry.begin();
      _cli.begin();

      _model.state.buzzer.push(BEEPER_SYSTEM_INIT);

      return 1;
    }

    int update()
    {
      //return 0;
      _hardware.update();

      _model.state.gyroUpdate = _model.state.gyroTimer.check();
      if(_model.state.gyroUpdate)
      {
        PIN_DEBUG(true);
        _sensor.update();
        PIN_DEBUG(false);
      }

      _model.state.loopUpdate = _model.state.gyroUpdate && _model.state.loopTimer.syncTo(_model.state.gyroTimer);
      if(_model.state.loopUpdate)
      {
        PIN_DEBUG(true);
        _input.update();
        PIN_DEBUG(false);
        _model.state.actuatorUpdate = _model.state.actuatorTimer.check();
        if(_model.state.actuatorUpdate)
        {
          PIN_DEBUG(true);
          _actuator.update();
          PIN_DEBUG(false);
        }

        PIN_DEBUG(true);
        _controller.update();
        PIN_DEBUG(false);
      }

      _model.state.mixerUpdate = _model.state.loopUpdate && _model.state.mixerTimer.syncTo(_model.state.loopTimer);
      if(_model.state.mixerUpdate)
      {
        PIN_DEBUG(true);
        _mixer.update();
        PIN_DEBUG(false);
      }

      if(_model.state.gyroUpdate)
      {
        PIN_DEBUG(true);
        _sensor.updateDelayed();
        PIN_DEBUG(false);
      }

      if(_model.state.loopUpdate && _model.blackboxEnabled())
      {
        PIN_DEBUG(true);
        _blackbox.update();
        PIN_DEBUG(false);
      }

      _model.state.telemetryUpdate = _model.config.telemetry && _model.state.telemetryTimer.check();
      if(_model.state.telemetryUpdate)
      {
        _telemetry.update();
      }

      PIN_DEBUG(true);
      _cli.update();
      _buzzer.update();
      PIN_DEBUG(false);

      if(_model.state.stats.timer.check())
      {
        _model.state.stats.calculate();
      }

      return 1;
    }

  private:
    Model _model;
    Hardware _hardware;
    Controller _controller;
    Input _input;
    Actuator _actuator;
    Sensor _sensor;
    Mixer _mixer;
    Blackbox _blackbox;
    Telemetry _telemetry;
    Cli _cli;
    Buzzer _buzzer;
};

}

#endif
