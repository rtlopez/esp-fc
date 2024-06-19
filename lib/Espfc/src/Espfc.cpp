#include "Espfc.h"
#include "Debug_Espfc.h"

namespace Espfc {

Espfc::Espfc():
  _hardware(_model), _controller(_model), _input(_model), _actuator(_model), _sensor(_model),
  _mixer(_model), _blackbox(_model)
#ifdef ESPFC_BUZER
  , _buzzer(_model)
#endif
  , _serial(_model)
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
  _serial.begin();      // requires _model.load()
  _model.logStorageResult();
  _hardware.begin();    // requires _model.load()
  _model.begin();       // requires _hardware.begin()
  _mixer.begin();
  _sensor.begin();      // requires _hardware.begin()
  _input.begin();       // requires _serial.begin()
  _actuator.begin();    // requires _model.begin()
  _controller.begin();
  _blackbox.begin();    // requires _serial.begin(), _actuator.begin()
#ifdef ESPFC_BUZER
  _buzzer.begin();
#endif
  _model.state.buzzer.push(BUZZER_SYSTEM_INIT);

  return 1;
}

int FAST_CODE_ATTR Espfc::update(bool externalTrigger)
{
  if(externalTrigger)
  {
    _model.state.gyroTimer.update();
  }
  else
  {
    if(!_model.state.gyroTimer.check()) return 0;
  }
  Stats::Measure measure(_model.state.stats, COUNTER_CPU_0);

#if defined(ESPFC_MULTI_CORE)

  _sensor.read();
  if(_model.state.inputTimer.syncTo(_model.state.gyroTimer, 1u))
  {
    _input.update();
  }
  if(_model.state.actuatorTimer.check())
  {
    _actuator.update();
  }

  _serial.update();
#ifdef ESPFC_BUZER
  _buzzer.update();
#endif
  _model.state.stats.update();

#else

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

  _serial.update();
#ifdef ESPFC_BUZER
  _buzzer.update();
#endif
  _model.state.stats.update();
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
      _sensor.preLoop();
      _controller.update();
      // skip mixer and bb if earlier than half cycle, possible delay in previous iteration, 
      // to keep space to receive dshot erpm frame, but process rest
      if(_loop_next < micros())
      {
        _loop_next = micros() + _model.state.loopTimer.interval / 2;
        _mixer.update();
        _blackbox.update();
      }
      _sensor.postLoop();
      break;
    case EVENT_ACCEL_READ:
      _sensor.fusion();
      break;
    default:
      break;
      // nothing
  }
#endif

  return 1;
}

}

