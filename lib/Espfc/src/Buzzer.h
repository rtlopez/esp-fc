#ifndef _ESPFC_BUZZER_H_
#define _ESPFC_BUZZER_H_

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
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
      if(_model.config.pin[PIN_BUZZER] == -1) return 0;
#ifndef UNIT_TEST
      pinMode(_model.config.pin[PIN_BUZZER], OUTPUT);
      digitalWrite(_model.config.pin[PIN_BUZZER], _model.config.buzzer.inverted);
#endif
      _model.state.buzzer.timer.setRate(100);

      return 1;
    }

    int update()
    {
      //_model.state.debug[0] = _e;
      //_model.state.debug[1] = _status;
      //_model.state.debug[2] = (int16_t)(millis() - _wait);

      if(_model.config.buzzer.pin == -1) return 0;
      if(!_model.state.buzzer.timer.check()) return 0;
      if(_wait > millis()) return 0;

      switch(_status)
      {
        case BUZZER_STATUS_IDLE: // wait for command
          if(!_model.state.buzzer.empty())
          {
            _e = _model.state.buzzer.pop();
            _scheme = schemes()[_e];
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
      digitalWrite(_model.config.pin[PIN_BUZZER], _model.config.buzzer.inverted ? !v : v);
    }

    void _delay(int time)
    {
      _wait = millis() + time * 10;
    }

    static const uint8_t** schemes()
    {
      static const uint8_t beeperSilence[] = { 0 };
      static const uint8_t beeperGyroCalibrated[] = { 10, 10, 10, 0 };
      static const uint8_t beeperRxLost[] = { 30, 0 };
      static const uint8_t beeperDisarming[] = { 10, 10, 0 };
      static const uint8_t beeperArming[] = { 20, 0 };
      static const uint8_t beeperSystemInit[] = { 10, 0 };
      static const uint8_t beeperBatteryLow[] = { 30, 0 };
      static const uint8_t beeperBatteryCritical[] = { 50, 0 };

      static const uint8_t* beeperSchemes[] = {
        //BEEPER_SILENCE
        beeperSilence,
        //BEEPER_GYRO_CALIBRATED,
        beeperGyroCalibrated,
        //BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
        beeperRxLost,
        //BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
        beeperSilence,
        //BEEPER_DISARMING,               // Beep when disarming the board
        beeperDisarming,
        //BEEPER_ARMING,                  // Beep when arming the board
        beeperArming,
        //BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
        beeperSilence,
        //BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
        beeperBatteryCritical,
        //BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
        beeperBatteryLow,
        //BEEPER_GPS_STATUS,              // FIXME **** Disable beeper when connected to USB ****
        beeperSilence,
        //BEEPER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
        beeperRxLost,
        //BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
        beeperSilence,
        //BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
        beeperSilence,
        //BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
        beeperSilence,
        //BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
        beeperSilence,
        //BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
        beeperSilence,
        //BEEPER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
        beeperSilence,
        //BEEPER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
        beeperSystemInit,
        //BEEPER_USB,                     // Some boards have beeper powered USB connected
        beeperSilence,
        //BEEPER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
        beeperSilence,
        //BEEPER_CRASH_FLIP_MODE,         // Crash flip mode is active
        beeperSilence,
        //BEEPER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
        beeperSilence,
        //BEEPER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
        beeperSilence,
        //BEEPER_ALL,                     // Turn ON or OFF all beeper conditions
        beeperSilence,
        //BEEPER_PREFERENCE,              // Save preferred beeper configuration
        beeperSilence
        // BEEPER_ALL and BEEPER_PREFERENCE must remain at the bottom of this enum
      };

      return beeperSchemes;
    }

    Model& _model;

    BuzzerPlayStatus _status;
    uint32_t _wait;
    const uint8_t * _scheme;
    BuzzerEvent _e;
};

}

#endif
