#pragma once

#include "Model.h"
#include "Control/Rates.h"

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
    float calculateSetpointRate(int axis, float input);

  private:
    Model& _model;
    Rates _rates;
    Utils::Filter _speedFilter;
};

}

}
