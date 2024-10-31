#pragma once

#include "Model.h"
#include "BaseSensor.h"
#include "Filter.h"

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
    Filter _vFilterFast;
    Filter _vFilter;
    Filter _iFilterFast;
    Filter _iFilter;
    State _state;
};

}

}
