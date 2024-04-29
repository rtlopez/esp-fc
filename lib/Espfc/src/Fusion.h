#pragma once

#include "Model.h"
#include "Madgwick.h"
#include "Mahony.h"

namespace Espfc {

class Fusion
{
  public:
    Fusion(Model& model);
    int begin();
    void restoreGain();
    int update();

    void experimentalFusion();
    void simpleFusion();
    void kalmanFusion();
    void complementaryFusion();
    void complementaryFusionOld();
    void rtqfFusion();
    void updatePoseFromAccelMag();

    // experimental
    void lerpFusion();
    void madgwickFusion();
    void mahonyFusion();

  private:
    Model& _model;
    bool _first;
    Madgwick _madgwick;
    Mahony _mahony;
};

}
