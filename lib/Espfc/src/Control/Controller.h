#pragma once

#include "Model.h"
#include "Control/Rates.h"
#include "Control/Altitude.hpp"

namespace Espfc::Control {

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
    void beginAltHold();
    void beginInnerLoop(size_t axis);
    void beginOuterLoop(size_t axis);

    Model& _model;
    Rates _rates;
    Utils::Filter _speedFilter;
};

}
