#ifndef _ESPFC_ESPFC_H_
#define _ESPFC_ESPFC_H_

#include "Target/Target.h"
#include "Model.h"
#include "Hardware.h"
#include "Controller.h"
#include "Input.h"
#include "Actuator.h"
#include "SensorManager.h"
#include "SerialManager.h"
#include "Fusion.h"
#include "Output/Mixer.h"
#include "Blackbox.h"
#include "Cli.h"
#include "Buzzer.h"

namespace Espfc {

class Espfc
{
  public:
    Espfc():
      _hardware(_model), _controller(_model), _input(_model), _actuator(_model), _sensor(_model),
      _mixer(_model), _blackbox(_model), _telemetry(_model), _buzzer(_model), _serial(_model)
      {}

    int load()
    {
      PIN_DEBUG_INIT();
      _model.load();
      _model.state.ioQueue.begin();
      _model.state.appQueue.begin();
      return 1;
    }

    int begin()
    {
      _hardware.begin();
      _model.begin();
      _mixer.begin();
      _sensor.begin();
      _input.begin();
      _actuator.begin();
      _controller.begin();
      _blackbox.begin();
      _model.state.buzzer.push(BEEPER_SYSTEM_INIT);

      return 1;
    }

    int beginOther()
    {
      _serial.begin();
      _model.logStorageResult();
      _buzzer.begin();
      return 1;
    }

    int update()
    {
      if(_model.state.gyroTimer.check()) {
        //_model.state.notify(Event(EVENT_START));
        _sensor.read();
      }

      if(_model.state.ioQueue.isEmpty()) {
        return 0;
      }

      Event e = _model.state.ioQueue.reveive();
      Serial2.write((uint8_t)e.type);

      //_sensor.onIoEvent(e);
      _input.onIoEvent(e);
      _actuator.onIoEvent(e);
      _serial.onIoEvent(e);

      /*if(_model.state.gyroTimer.check())
      {
        _sensor.update();
        if(_model.state.loopTimer.syncTo(_model.state.gyroTimer))
        {
          _controller.update();
          if(_model.state.mixerTimer.syncTo(_model.state.loopTimer))
          {
            _mixer.update();
          }
          _input.update();
          if(_model.state.actuatorTimer.check())
          {
            _actuator.update();
          }
          _blackbox.update();
        }
        _sensor.updateDelayed();
      }*/

      return 1;
    }

    // other task
    int updateOther()
    {
      if(_model.state.appQueue.isEmpty()) {
        return 1;
      }

      Event e = _model.state.appQueue.reveive();
      //Serial2.write((uint8_t)e.type);

      _sensor.onAppEvent(e);
      _controller.onAppEvent(e);
      _mixer.onAppEvent(e);
      _blackbox.onAppEvent(e);

      /*if(_model.state.serialTimer.check())
      {
        _serial.update();
      }*/
      _buzzer.update();
      _model.state.stats.update();

      return 1;
    }

  private:
    Model _model;
    Hardware _hardware;
    Controller _controller;
    Input _input;
    Actuator _actuator;
    SensorManager _sensor;
    Output::Mixer _mixer;
    Blackbox _blackbox;
    Telemetry _telemetry;
    Buzzer _buzzer;
    SerialManager _serial;
};

}

#endif
