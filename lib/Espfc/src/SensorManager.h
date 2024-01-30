#ifndef _ESPFC_SENSOR_H_
#define _ESPFC_SENSOR_H_

#include <Arduino.h>
#include "Model.h"
#include "Filter.h"
#include "Fusion.h"
#include "Hardware.h"
#include "Sensor/GyroSensor.h"
#include "Sensor/AccelSensor.h"
#include "Sensor/MagSensor.h"
#include "Sensor/BaroSensor.h"
#include "Sensor/VoltageSensor.h"

namespace Espfc {

class SensorManager
{
  public:
    SensorManager(Model& model): _model(model), _gyro(model), _accel(model), _mag(model), _baro(model), _voltage(model), _fusion(model), _fusionUpdate(false) {}

    int begin()
    {
      _gyro.begin();
      _accel.begin();
      _mag.begin();
      _baro.begin();
      _voltage.begin();
      _fusion.begin();
      
      return 1;
    }

    int read()
    {
      _gyro.read();
      if(_model.state.loopTimer.syncTo(_model.state.gyroTimer))
      {
        _model.state.appQueue.send(Event(EVENT_GYRO_READ));
      }

      int status = _accel.update();
      if(status)
      {
        _model.state.appQueue.send(Event(EVENT_ACCEL_READ));
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

      return 1;
    }

    int preLoop()
    {
      _gyro.filter();
      if(_model.state.gyroBiasSamples == 0)
      {
        _model.state.gyroBiasSamples = -1;
        _fusion.restoreGain();
      }
      return 1;
    }

    int postLoop()
    {
      _gyro.dynNotchAnalyze();
      return 1;
    }

    int fusion()
    {
      return _fusion.update();
    }

    // main task
    int update()
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
    int updateDelayed()
    {
      _gyro.dynNotchAnalyze();
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

    Model& _model;
    Sensor::GyroSensor _gyro;
    Sensor::AccelSensor _accel;
    Sensor::MagSensor _mag;
    Sensor::BaroSensor _baro;
    Sensor::VoltageSensor _voltage;
    Fusion _fusion;
    bool _fusionUpdate;
};

}

#endif
