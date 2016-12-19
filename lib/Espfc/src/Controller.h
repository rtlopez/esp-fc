#ifndef _ESPFC_CONTROLLER_H_
#define _ESPFC_CONTROLLER_H_

#include "Model.h"

namespace Espfc {

class Controller
{
  public:
    Controller(Model& model): _model(model) {}
    int begin() {}
    int update() {}

  private:
    Model& _model;
};

}

#endif
