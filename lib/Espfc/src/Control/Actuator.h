#pragma once

#include "Model.h"

namespace Espfc {

namespace Control {

class Actuator
{
  public:
    Actuator(Model& model);

    int begin();
    int update();

  #ifndef UNIT_TEST
  private:
  #endif

    void updateScaler();
    void updateArmingDisabled();
    void updateModeMask();
    bool canActivateMode(FlightMode mode);
    void updateArmed();
    void updateAirMode();
    void updateBuzzer();
    void updateDynLpf();
    void updateRescueConfig();

    Model& _model;
};

}

}
