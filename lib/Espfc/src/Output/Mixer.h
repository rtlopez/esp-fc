#pragma once

#include "Model.h"
#include "EscDriver.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Output {

class Mixer
{
  public:
    Mixer(Model& model);
    int begin();
    int update() FAST_CODE_ATTR;

    void updateMixer(const MixerConfig& mixer, float * outputs) FAST_CODE_ATTR;
    float limitThrust(float thrust, ThrottleLimitType type, int8_t limit) FAST_CODE_ATTR;
    float limitOutput(float output, const OutputChannelConfig& occ, int limit) FAST_CODE_ATTR;
    void writeOutput(const MixerConfig& mixer, float * out) FAST_CODE_ATTR;
    void readTelemetry() FAST_CODE_ATTR;
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
