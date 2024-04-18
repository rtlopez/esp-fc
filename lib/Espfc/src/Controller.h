#ifndef _ESPFC_CONTROLLER_H_
#define _ESPFC_CONTROLLER_H_

#include "Model.h"
#include "Control/Rates.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

class Controller
{
  public:
    Controller(Model& model);
    int begin();
    int update() FAST_CODE_ATTR;

    void outerLoopRobot();
    void innerLoopRobot();
    void outerLoop() FAST_CODE_ATTR;
    void innerLoop() FAST_CODE_ATTR;

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
