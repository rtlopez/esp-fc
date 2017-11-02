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
    Buzzer(Model& model): _model(model) {}

    int begin()
    {
      if(_model.config.buzzer.pin == -1) return 0;

      pinMode(_model.config.buzzer.pin, OUTPUT);

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
            BuzzerEvent e = _model.state.buzzer.pop();
            if(_model.config.buzzer.beeperMask & 1 << (e - 1))
            {
              _schemePtr = schemes[e];
              _status = BUZZER_STATUS_TEST;
            }
          }
          break;
        case BUZZER_STATUS_TEST: // test for end or continue
          if(!_schemePtr || *_schemePtr == 0)
          {
            _play(false, 50, BUZZER_STATUS_IDLE);
          }
          else
          {
            _play(true, *_schemePtr, BUZZER_STATUS_ON); // start playing
          }
          break;
        case BUZZER_STATUS_ON: // end playing with pause, same length as on
          _play(false, *_schemePtr, BUZZER_STATUS_OFF);
          break;
        case BUZZER_STATUS_OFF: // move to next cycle
          _schemePtr++;
          _status = BUZZER_STATUS_TEST;
          break;
      }

      return 1;
    }

    void _play(bool v, uint8_t time, BuzzerPlayStatus s)
    {
      digitalWrite(_model.config.buzzer.pin, v);
      _wait = millis() + time * 10;
      _status = s;
    }

    const uint8_t * _schemePtr;
    BuzzerPlayStatus _status;
    uint32_t _wait;

    static const uint8_t* schemes[];

    Model& _model;
};

}

#endif
