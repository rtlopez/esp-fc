#ifndef _ESPFC_SENSOR_BARO_SENSOR_H_
#define _ESPFC_SENSOR_BARO_SENSOR_H_

#include "BaseSensor.h"
#include "Model.h"
#include "Device/BaroDevice.h"
#include "Filter.h"

namespace Espfc {

namespace Sensor {

class BaroSensor: public BaseSensor
{
  public:
    enum BaroState
    {
      BARO_STATE_INIT,
      BARO_STATE_TEMP_GET,
      BARO_STATE_PRESS_GET,
    };

    BaroSensor(Model& model);

    int begin();
    int update();
    int read();

  private:
    void readTemperature();
    void readPressure();
    void updateAltitude();

    Model& _model;
    Device::BaroDevice * _baro;
    BaroState _state;
    Filter _temperatureFilter;
    Filter _pressureFilter;
    Filter _altitudeFilter;
    uint32_t _wait;
    int32_t _counter;
};

}

}
#endif