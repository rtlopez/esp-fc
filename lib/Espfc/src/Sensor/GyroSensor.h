#pragma once

#include "BaseSensor.h"
#include "Model.h"
#include "Device/GyroDevice.h"
#include "Math/Sma.h"
#ifdef ESPFC_DSP
#include "Math/FFTAnalyzer.h"
#else
#include "Math/FreqAnalyzer.h"
#endif

#define ESPFC_FUZZY_ACCEL_ZERO 0.05
#define ESPFC_FUZZY_GYRO_ZERO  0.20

namespace Espfc {

namespace Sensor {

class GyroSensor: public BaseSensor
{
  public:
    GyroSensor(Model& model);

    int begin();
    int read();
    int filter();
    void postLoop();
    void rpmFilterUpdate();
    void dynNotchFilterUpdate();

  private:
    void calibrate();

    Math::Sma<VectorFloat, 8> _sma;
    Math::Sma<VectorFloat, 8> _dyn_notch_sma;
    size_t _dyn_notch_denom;
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
    Math::FFTAnalyzer<128> _fft[3];
#else
    Math::FreqAnalyzer _freqAnalyzer[3];
#endif

};

}

}
