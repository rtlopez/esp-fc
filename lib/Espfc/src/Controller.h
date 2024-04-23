#ifndef _ESPFC_CONTROLLER_H_
#define _ESPFC_CONTROLLER_H_

#include "Model.h"
#include "Control/Rates.h"

namespace Espfc {

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
    Filter _speedFilter;
};

}

#endif
