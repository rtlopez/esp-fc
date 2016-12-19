#ifndef _ESPFC_MIXER_H_
#define _ESPFC_MIXER_H_

#include "Model.h"

namespace Espfc {

class Mixer
{
  public:
    Mixer(Model& model): _model(model) {}
    int begin() {}
    int update() {}

  private:
    Model& _model;

};

}

#endif
