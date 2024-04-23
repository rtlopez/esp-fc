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

int FAST_CODE_ATTR SensorManager::read()
{
  _gyro.read();

  if(_model.state.loopTimer.syncTo(_model.state.gyroTimer))
  {
    _model.state.appQueue.send(Event(EVENT_GYRO_READ));
  }

  if(_model.state.accelTimer.syncTo(_model.state.gyroTimer))
  {
    _accel.update();
    _model.state.appQueue.send(Event(EVENT_ACCEL_READ));
    return 1;
  }

  if(_mag.update()) return 1;

  if(_baro.update()) return 1;

  if(_voltage.update()) return 1;

  return 0;
}

int FAST_CODE_ATTR SensorManager::preLoop()
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

int FAST_CODE_ATTR SensorManager::fusion()
{
  return _fusion.update();
}

// main task
int FAST_CODE_ATTR SensorManager::update()
{
  _gyro.read();
  return preLoop();
}

// sub task
int SensorManager::updateDelayed()
{
  _gyro.postLoop();

  // update at most one sensor besides gyro
  int status = 0;
  if(_model.state.accelTimer.syncTo(_model.state.gyroTimer))
  {
    _accel.update();
    status = 1;
  }

  // delay imu update to next cycle
  if(_fusionUpdate)
  {
    _fusionUpdate = false;
    _fusion.update();
  }
  _fusionUpdate = status;

  if(status) return 1;

  if(_mag.update()) return 1;

  if(_baro.update()) return 1;

  if(_voltage.update()) return 0;

  return 0;
}

}
