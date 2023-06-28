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
    SensorManager(Model& model): _model(model), _gyro(model), _accel(model), _mag(model), _baro(model), _voltage(model), _fusion(model) {}

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

    int onAppEvent(const Event& e)
    {
      switch(e.type)
      {
        case EVENT_GYRO_READ:
          _gyro.filter();
          if(_model.state.gyroBiasSamples == 0)
          {
            _model.state.gyroBiasSamples = -1;
            _fusion.restoreGain();
          }
          return 1;
        case EVENT_ACCEL_READ:
          _accel.filter();
          _model.state.imuUpdate = true;
          return 1;
        case EVENT_MAG_READ:
          _mag.filter();
          _model.state.imuUpdate = true;
          return 1;
        case EVENT_SENSOR_READ:
          if(_model.state.imuUpdate)
          {
            _fusion.update();
            _model.state.imuUpdate = false;
          }
          _model.state.appQueue.send(Event(EVENT_IMU_UPDATE));
          return 1;
        default:
          break;
      }
      return 0;
    }

    int read()
    {
      _model.state.appQueue.send(Event(EVENT_START));

      _gyro.read();
      _model.state.appQueue.send(Event(EVENT_GYRO_READ));

      if(_accel.read())
      {
        _model.state.appQueue.send(Event(EVENT_ACCEL_READ));
      }

      if(_mag.read())
      {
        _model.state.appQueue.send(Event(EVENT_MAG_READ));
      }

      if(_baro.update())
      {
        _model.state.appQueue.send(Event(EVENT_BARO_READ));
      }

      if (_voltage.update())
      {
        _model.state.appQueue.send(Event(EVENT_VOLTAGE_READ));
      }

      _model.state.appQueue.send(Event(EVENT_SENSOR_READ));

      return 1;
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
      int status = _accel.update();
      if(!status)
      {
        status = _mag.update();
      }

      if(status)
      {
        _fusion.update();
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
};

}

#endif
