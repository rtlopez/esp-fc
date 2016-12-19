#ifndef _ESPFC_INPUT_H_
#define _ESPFC_INPUT_H_

#include "Model.h"

namespace Espfc {

class Input
{
  public:
    Input(Model& model): _model(model) {}
    int begin() {}
    int update() {}

  private:
    Model& _model;

};

}

#endif
