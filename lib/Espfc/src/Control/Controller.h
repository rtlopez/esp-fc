#pragma once

#include "Model.h"
#include "Control/Rates.h"
#include "Control/Altitude.hpp"

namespace Espfc {

namespace Control {

class Controller
{
  public:
    Controller(Model& model);
    int begin();
    int update();

    void outerLoopRobot();
    void innerLoopRobot();
    void outerLoop();
    void innerLoop();

    inline float getTpaFactor() const;
    inline void resetIterm();
    float calculateSetpointRate(int axis, float input) const;
    float calcualteAltHoldSetpoint() const;

  private:
    Model& _model;
    Altitude _altitude;
    Rates _rates;
    Utils::Filter _speedFilter;
};

}

}
