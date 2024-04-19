#include "SensorManager.h"

namespace Espfc {

SensorManager::SensorManager(Model& model): _model(model), _gyro(model), _accel(model), _mag(model), _baro(model), _voltage(model), _fusion(model), _fusionUpdate(false) {}

int SensorManager::begin()
{
  _gyro.begin();
  _accel.begin();
  _mag.begin();
  _baro.begin();
  _voltage.begin();
  _fusion.begin();
  
  return 1;
}

int SensorManager::read()
{
  int ret = 0;
  _gyro.read();
  if(_model.state.loopTimer.syncTo(_model.state.gyroTimer))
  {
    ret = 1;
    _model.state.appQueue.send(Event(EVENT_GYRO_READ));
  }

  int status = 0;
  if(_model.state.accelTimer.syncTo(_model.state.gyroTimer))
  {
    status = _accel.update();
    if(status)
    {
      _model.state.appQueue.send(Event(EVENT_ACCEL_READ));
    }
  }

  if (!status)
  {
    status = _mag.update();
  }

  if(!status)
  {
    status = _baro.update();
  }

  if(!status)
  {
    status = _voltage.update();
  }

  return ret;
}

int SensorManager::preLoop()
{
  _gyro.filter();
  if(_model.state.gyroBiasSamples == 0)
  {
    _model.state.gyroBiasSamples = -1;
    _fusion.restoreGain();
  }
  return 1;
}

int SensorManager::postLoop()
{
  _gyro.postLoop();
  return 1;
}

int SensorManager::fusion()
{
  return _fusion.update();
}

// main task
int SensorManager::update()
{
  int status = _gyro.update();

  if(_model.state.gyroBiasSamples == 0)
  {
    _model.state.gyroBiasSamples = -1;
    _fusion.restoreGain();
  }

  return status;
}

// sub task
int SensorManager::updateDelayed()
{
  _gyro.postLoop();
  int status = _accel.update();
  if(_fusionUpdate)
  {
    _fusionUpdate = false;
    _fusion.update();
  }
  _fusionUpdate = status; // update in next loop cycle

  if(!status)
  {
    status = _mag.update();
  }

  if(!status)
  {
    status = _baro.update();
  }

  if(!status)
  {
    _voltage.update();
  }

  return status;
}

}
