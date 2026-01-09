#ifndef _ESPFC_SENSOR_BARO_SENSOR_H_
#define _ESPFC_SENSOR_BARO_SENSOR_H_

#include "BaseSensor.h"
#include "Model.h"
#include "Device/BaroDevice.h"
#include "Utils/Filter.h"

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
    Utils::Filter _temperatureFilter;
    Utils::Filter _temperatureMedianFilter;
    Utils::Filter _pressureFilter;
    Utils::Filter _pressureMedianFilter;
    Utils::Filter _altitudeFilter;
    Utils::Filter _varioFilter;
    uint32_t _wait;
    int32_t _counter;
};

}

}
#endif