#pragma once

#include "Model.h"
#include "BaseSensor.h"
#include "Utils/Filter.h"

namespace Espfc {

namespace Sensor {

class VoltageSensor: public BaseSensor
{
  public:
    enum State {
      VBAT,
      IBAT
    };
    VoltageSensor(Model& model);
    int begin();
    int update();
    int readVbat();
    int readIbat();

  private:
    Model& _model;
    Utils::Filter _vFilterFast;
    Utils::Filter _vFilter;
    Utils::Filter _iFilterFast;
    Utils::Filter _iFilter;
    State _state;
};

}

}
