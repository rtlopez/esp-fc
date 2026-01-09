#pragma once

#include "BaseSensor.h"
#include "Model.h"
#include "Device/GyroDevice.h"
#include "Utils/Sma.hpp"
#ifdef ESPFC_DSP
#include "Utils/FFTAnalyzer.hpp"
#else
#include "Utils/FreqAnalyzer.hpp"
#endif

namespace Espfc {

namespace Sensor {

class GyroSensor: public BaseSensor
{
  public:
    GyroSensor(Model& model);
    ~GyroSensor();

    int begin();
    int read();
    int filter();
    void postLoop();
    void rpmFilterUpdate();
    void dynNotchFilterUpdate();

  private:
    void calibrate();

    Utils::Sma<VectorFloat, 8> _sma;
    Utils::Sma<VectorFloat, 8> _dyn_notch_sma;
    size_t _dyn_notch_denom;
    size_t _dyn_notch_count;
    bool _dyn_notch_enabled;
    bool _dyn_notch_debug;
    bool _rpm_enabled;
    size_t _rpm_motor_index;
    float _rpm_weights[3];
    float _rpm_fade_inv;
    float _rpm_min_freq;
    float _rpm_max_freq;
    float _rpm_q;

    Model& _model;
    Device::GyroDevice * _gyro;

#ifdef ESPFC_DSP
    Utils::FFTAnalyzer<128> _fft[3];
#else
    Utils::FreqAnalyzer _freqAnalyzer[3];
#endif

};

}

}
