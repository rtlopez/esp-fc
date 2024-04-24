#include "Espfc.h"

namespace Espfc {

Espfc::Espfc():
  _hardware(_model), _controller(_model), _input(_model), _actuator(_model), _sensor(_model),
  _mixer(_model), _blackbox(_model), _buzzer(_model), _serial(_model)
  {}

int Espfc::load()
{
  PIN_DEBUG_INIT();
  _model.load();
  _model.state.appQueue.begin();
  return 1;
}

int Espfc::begin()
{
  _serial.begin();
  _model.logStorageResult();
  _hardware.begin();
  _model.begin();
  //_mixer.begin();
  _sensor.begin();
  _input.begin();
  _actuator.begin();
  _controller.begin();
  _blackbox.begin();

  return 1;
}

int Espfc::beginOther()
{
  _mixer.begin();
  _buzzer.begin();
  _model.state.buzzer.push(BEEPER_SYSTEM_INIT);
  return 1;
}

int FAST_CODE_ATTR Espfc::update()
{
#if defined(ESPFC_MULTI_CORE)
  if(!_model.state.gyroTimer.check())
  {
    return 0;
  }
  Stats::Measure measure(_model.state.stats, COUNTER_CPU_0);

  _sensor.read();
  if(_model.state.inputTimer.syncTo(_model.state.gyroTimer, 1u))
  {
    PIN_DEBUG(HIGH);
    _input.update();
    PIN_DEBUG(LOW);
  }
  if(_model.state.actuatorTimer.check())
  {
    _actuator.update();
  }
  if(_model.state.serialTimer.check())
  {
    _serial.update();
    _buzzer.update();
    _model.state.stats.update();
  }

  return 1;
#else
  if(_model.state.gyroTimer.check())
  {
    Stats::Measure measure(_model.state.stats, COUNTER_CPU_0);
    _sensor.update();
    if(_model.state.loopTimer.syncTo(_model.state.gyroTimer))
    {
      _controller.update();
      if(_model.state.mixerTimer.syncTo(_model.state.loopTimer))
      {
        _mixer.update();
      }
      _blackbox.update();
      if(_model.state.inputTimer.syncTo(_model.state.gyroTimer, 1u))
      {
        _input.update();
      }
      if(_model.state.actuatorTimer.check())
      {
        _actuator.update();
      }
    }
    _sensor.updateDelayed();
  }
#endif

  return 1;
}

// other task
int FAST_CODE_ATTR Espfc::updateOther()
{
#if defined(ESPFC_MULTI_CORE)
  if(_model.state.appQueue.isEmpty())
  {
    return 0;
  }
  Event e = _model.state.appQueue.receive();

  Stats::Measure measure(_model.state.stats, COUNTER_CPU_1);

  switch(e.type)
  {
    case EVENT_GYRO_READ:
      // TODO: skip if too quick
      //PIN_DEBUG(HIGH);
      _sensor.preLoop();
      //PIN_DEBUG(LOW);
      _controller.update();
      //PIN_DEBUG(HIGH);
      _mixer.update();
      //PIN_DEBUG(LOW);
      _blackbox.update();
      //PIN_DEBUG(HIGH);
      _sensor.postLoop();
      //PIN_DEBUG(LOW);
      break;
    case EVENT_ACCEL_READ:
      //PIN_DEBUG(HIGH);
      _sensor.fusion();
      //PIN_DEBUG(LOW);
      break;
    default:
      break;
      // nothing
  }
#else
  if(_model.state.serialTimer.check())
  {
    Stats::Measure measure(_model.state.stats, COUNTER_CPU_1);
    _serial.update();
    _buzzer.update();
    _model.state.stats.update();
  }
#endif

  return 1;
}

}

