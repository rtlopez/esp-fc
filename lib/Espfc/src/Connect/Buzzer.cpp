#include "Buzzer.hpp"
#include "Hal/Gpio.h"
#include "driver/ledc.h"

namespace Espfc {

namespace Connect {

Buzzer::Buzzer(Model& model): _model(model), _status(BUZZER_STATUS_IDLE), _wait(0), _scheme(NULL), _e(BUZZER_SILENCE) {}

int Buzzer::begin()
{
  if(_model.config.pin[PIN_BUZZER] == -1) return 0;

  switch(_model.config.buzzer.type)
  {
    case 0:
      Hal::Gpio::pinMode(_model.config.pin[PIN_BUZZER], OUTPUT);
      Hal::Gpio::digitalWrite(_model.config.pin[PIN_BUZZER], (pin_status_t)_model.config.buzzer.inverted);
      break;
    case 1:
      ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = static_cast<uint32_t>(_model.config.buzzer.frequency),
        .clk_cfg = LEDC_AUTO_CLK
      };
      ledc_timer_config(&ledc_timer);

      ledc_channel_config_t ledc_channel = {
        .gpio_num = _model.config.pin[PIN_BUZZER],
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
      };
      ledc_channel_config(&ledc_channel);

      gpio_set_drive_capability((gpio_num_t)_model.config.pin[PIN_BUZZER], GPIO_DRIVE_CAP_3);
      break;
  }

  _model.state.buzzer.timer.setRate(100);

  return 1;
}

int Buzzer::update()
{
  //_model.state.debug[0] = _e;
  //_model.state.debug[1] = _status;
  //_model.state.debug[2] = (int16_t)(millis() - _wait);

  if(_model.config.pin[PIN_BUZZER] == -1) return 0;
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
        _e = BUZZER_SILENCE;
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

void Buzzer::_play(bool v, int time, BuzzerPlayStatus s)
{
  _write(v);
  _delay(time);
  _status = s;
}

void Buzzer::_write(bool v)
{
  switch(_model.config.buzzer.type)
  {
    case 0:
      Hal::Gpio::digitalWrite(_model.config.pin[PIN_BUZZER], (pin_status_t)(_model.config.buzzer.inverted ? !v : v));
      break;
    case 1:
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, v ? 4096 : 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
      break;
  }
}

void Buzzer::_delay(int time)
{
  _wait = millis() + time * 10;
}

const uint8_t** Buzzer::schemes()
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
    //BUZZER_SILENCE
    beeperSilence,
    //BUZZER_GYRO_CALIBRATED,
    beeperGyroCalibrated,
    //BUZZER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
    beeperRxLost,
    //BUZZER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
    beeperSilence,
    //BUZZER_DISARMING,               // Beep when disarming the board
    beeperDisarming,
    //BUZZER_ARMING,                  // Beep when arming the board
    beeperArming,
    //BUZZER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
    beeperSilence,
    //BUZZER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
    beeperBatteryCritical,
    //BUZZER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
    beeperBatteryLow,
    //BUZZER_GPS_STATUS,              // Num beeps = num Gps Sats, FIXME **** Disable beeper when connected to USB ****
    beeperSilence,
    //BUZZER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
    beeperRxLost,
    //BUZZER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
    beeperSilence,
    //BUZZER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
    beeperSilence,
    //BUZZER_READY_BEEP,              // Ring a tone when GPS is locked and ready
    beeperGyroCalibrated,
    //BUZZER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
    beeperSilence,
    //BUZZER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
    beeperSilence,
    //BUZZER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
    beeperSilence,
    //BUZZER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
    beeperSystemInit,
    //BUZZER_USB,                     // Some boards have beeper powered USB connected
    beeperSilence,
    //BUZZER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
    beeperSilence,
    //BUZZER_CRASH_FLIP_MODE,         // Crash flip mode is active
    beeperSilence,
    //BUZZER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
    beeperSilence,
    //BUZZER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
    beeperSilence,
    //BUZZER_ALL,                     // Turn ON or OFF all beeper conditions
    beeperSilence,
    //BUZZER_PREFERENCE,              // Save preferred beeper configuration
    beeperSilence
    // BUZZER_ALL and BUZZER_PREFERENCE must remain at the bottom of this enum
  };

  return beeperSchemes;
}

}

}
