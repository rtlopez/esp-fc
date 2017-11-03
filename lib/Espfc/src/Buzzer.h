#ifndef _ESPFC_BUZZER_H_
#define _ESPFC_BUZZER_H_

#include "Arduino.h"
#include "Model.h"

namespace Espfc {

enum BuzzerPlayStatus
{
  BUZZER_STATUS_IDLE,
  BUZZER_STATUS_TEST,
  BUZZER_STATUS_ON,
  BUZZER_STATUS_OFF
};

class Buzzer
{
  public:
    Buzzer(Model& model): _model(model), _status(BUZZER_STATUS_IDLE), _wait(0), _scheme(NULL), _e(BEEPER_SILENCE) {}

    int begin()
    {
      if(_model.config.buzzer.pin == -1) return 0;

      pinMode(_model.config.buzzer.pin, OUTPUT);
      digitalWrite(_model.config.buzzer.pin, _model.config.buzzer.inverted);
      _model.state.buzzer.timer.setRate(100);

      return 1;
    }

    int update()
    {
      if(_model.config.buzzer.pin == -1) return 0;
      if(!_model.state.buzzer.timer.check()) return 0;
      if(_wait > millis()) return 0;

      switch(_status)
      {
        case BUZZER_STATUS_IDLE: // wait for command
          if(!_model.state.buzzer.empty())
          {
            _e = _model.state.buzzer.pop();
            _scheme = schemes[_e];
            _status = BUZZER_STATUS_TEST;
          }
          break;
        case BUZZER_STATUS_TEST: // test for end or continue
          if(!_scheme || *_scheme == 0)
          {
            _play(false, 10, BUZZER_STATUS_IDLE);
            _scheme = NULL;
            _e = BEEPER_SILENCE;
          }
          else
          {
            _play(true, *_scheme, BUZZER_STATUS_ON); // start playing
          }
          break;
        case BUZZER_STATUS_ON: // end playing with pause, same length as on
          _play(false, (*_scheme) / 2, BUZZER_STATUS_OFF);
          break;
        case BUZZER_STATUS_OFF: // move to next cycle
          _scheme++;
          _status = BUZZER_STATUS_TEST;
          break;
      }

      return 1;
    }

    void _play(bool v, int time, BuzzerPlayStatus s)
    {
      _write(v);
      _delay(time);
      _status = s;
    }

    void _write(bool v)
    {
      digitalWrite(_model.config.buzzer.pin, _model.config.buzzer.inverted ? !v : v);
    }

    void _delay(int time)
    {
      _wait = millis() + time * 10;
    }

    Model& _model;

    BuzzerPlayStatus _status;
    uint32_t _wait;
    const uint8_t * _scheme;
    BuzzerEvent _e;

    static const uint8_t* schemes[];
};

}

#endif
