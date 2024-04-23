#pragma once

#include "Model.h"
#include "EscDriver.h"

namespace Espfc {

namespace Output {

class Mixer
{
  public:
    Mixer(Model& model);
    int begin();
    int update();

    void updateMixer(const MixerConfig& mixer, float * outputs);
    float limitThrust(float thrust, ThrottleLimitType type, int8_t limit);
    float limitOutput(float output, const OutputChannelConfig& occ, int limit);
    void writeOutput(const MixerConfig& mixer, float * out);
    void readTelemetry();
    float inline erpmToHz(float erpm);
    float inline erpmToRpm(float erpm);
    bool inline _stop(void);

  private:
    Model& _model;
    EscDriver * _motor;
    EscDriver * _servo;

    EscDriver escMotor;
    EscDriver escServo;
    uint32_t _statsCounter;
    uint32_t _statsCounterMax;
    float _erpmToHz;
};

}

}
