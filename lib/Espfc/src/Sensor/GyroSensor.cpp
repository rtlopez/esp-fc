
#include "GyroSensor.h"
#include "Utils/FilterHelper.h"
#include "Utils/Sma.ipp"
#ifdef ESPFC_DSP
#include "Utils/FFTAnalyzer.ipp"
#endif

namespace Espfc::Sensor
{

static constexpr float ESPFC_FUZZY_ACCEL_ZERO = 0.05f;
static constexpr float ESPFC_FUZZY_GYRO_ZERO = 0.20f;

GyroSensor::GyroSensor(Model &model) : _dyn_notch_denom(1), _model(model)
{
}

GyroSensor::~GyroSensor()
{
}

int GyroSensor::begin()
{
  _gyro = _model.state.gyro.dev;
  if (!_gyro) return 0;

  _gyro->setDLPFMode(_model.config.gyro.dlpf);
  _gyro->setRate(_gyro->getRate());
  _model.state.gyro.scale = Utils::toRad(2000.f) / 32768.f;

  _model.state.gyro.calibrationState = CALIBRATION_START; // calibrate gyro on start
  _model.state.gyro.calibrationRate = _model.state.loopTimer.rate;
  _model.state.gyro.biasAlpha = 5.0f / _model.state.gyro.calibrationRate;

  _sma.begin(_model.config.loopSync);
  _dyn_notch_denom = std::max((uint32_t)1, _model.state.loopTimer.rate / 1000);
  _dyn_notch_sma.begin(_dyn_notch_denom);
  _dyn_notch_count = std::min((size_t)_model.config.gyro.dynamicFilter.count, DYN_NOTCH_COUNT_MAX);
  _dyn_notch_enabled = _model.isFeatureActive(FEATURE_DYNAMIC_FILTER) && _dyn_notch_count > 0 && _model.state.loopTimer.rate >= DynamicFilterConfig::MIN_FREQ;
  _dyn_notch_debug = _model.config.debug.mode == DEBUG_FFT_FREQ || _model.config.debug.mode == DEBUG_FFT_TIME;

  _rpm_enabled = _model.config.gyro.rpmFilter.harmonics > 0 && _model.config.output.dshotTelemetry;
  _rpm_motor_index = 0;
  _rpm_fade_inv = 1.0f / _model.config.gyro.rpmFilter.fade;
  _rpm_min_freq = _model.config.gyro.rpmFilter.minFreq;
  _rpm_max_freq = 0.48f * _model.state.loopTimer.rate;
  _rpm_q = _model.config.gyro.rpmFilter.q * 0.01f;

  for (size_t i = 0; i < RPM_FILTER_HARMONICS_MAX; i++)
  {
    _rpm_weights[i] = Utils::clamp(0.01f * _model.config.gyro.rpmFilter.weights[i], 0.0f, 1.0f);
  }
  for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
#ifdef ESPFC_DSP
    _fft[i].begin(_model.state.loopTimer.rate / _dyn_notch_denom, _model.config.gyro.dynamicFilter, i);
#else
    _freqAnalyzer[i].begin(_model.state.loopTimer.rate / _dyn_notch_denom, _model.config.gyro.dynamicFilter);
#endif
  }

  _model.logger.info().log(F("GYRO INIT")).log(FPSTR(Device::GyroDevice::getName(_gyro->getType()))).log(_gyro->getAddress()).log(_model.config.gyro.dlpf).log(_gyro->getRate()).log(_model.state.gyro.timer.rate).logln(_model.state.gyro.timer.interval);

  return 1;
}

int FAST_CODE_ATTR GyroSensor::read()
{
  if (!_model.gyroActive()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_GYRO_READ);

  _gyro->readGyro(_model.state.gyro.raw);

  VectorFloat input = static_cast<VectorFloat>(_model.state.gyro.raw) * _model.state.gyro.scale;
  align(input, _model.config.gyro.align);
  input = _model.state.boardAlignment.apply(input);

  if (_model.config.gyro.filter3.freq)
  {
    _model.state.gyro.sampled = Utils::applyFilter(_model.state.gyro.filter3, input);
  }
  else
  {
    _model.state.gyro.sampled = _sma.update(input);
  }

  return 1;
}

int FAST_CODE_ATTR GyroSensor::filter()
{
  if (!_model.gyroActive()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_GYRO_FILTER);

  _model.state.gyro.adc = _model.state.gyro.sampled;

  calibrate();

  _model.state.gyro.scaled = _model.state.gyro.adc; // must be after calibration

  for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    _model.setDebug(DEBUG_GYRO_RAW, i, _model.state.gyro.raw[i]);
    _model.setDebug(DEBUG_GYRO_SCALED, i, lrintf(Utils::toDeg(_model.state.gyro.scaled[i])));
  }

  _model.setDebug(DEBUG_GYRO_SAMPLE, 0, lrintf(Utils::toDeg(_model.state.gyro.adc[_model.config.debug.axis])));

  _model.state.gyro.adc = Utils::applyFilter(_model.state.gyro.filter2, _model.state.gyro.adc);

  _model.setDebug(DEBUG_GYRO_SAMPLE, 1, lrintf(Utils::toDeg(_model.state.gyro.adc[_model.config.debug.axis])));

  if (_rpm_enabled)
  {
    for (size_t m = 0; m < RPM_FILTER_MOTOR_MAX; m++)
    {
      for (size_t n = 0; n < _model.config.gyro.rpmFilter.harmonics; n++)
      {
        _model.state.gyro.adc = Utils::applyFilter(_model.state.gyro.rpmFilter[m][n], _model.state.gyro.adc);
      }
    }
  }

  _model.setDebug(DEBUG_GYRO_SAMPLE, 2, lrintf(Utils::toDeg(_model.state.gyro.adc[_model.config.debug.axis])));

  _model.state.gyro.adc = Utils::applyFilter(_model.state.gyro.notch1Filter, _model.state.gyro.adc);
  _model.state.gyro.adc = Utils::applyFilter(_model.state.gyro.notch2Filter, _model.state.gyro.adc);
  _model.state.gyro.adc = Utils::applyFilter(_model.state.gyro.filter, _model.state.gyro.adc);

  _model.setDebug(DEBUG_GYRO_SAMPLE, 3, lrintf(Utils::toDeg(_model.state.gyro.adc[_model.config.debug.axis])));

  if (_dyn_notch_enabled || _dyn_notch_debug)
  {
    _model.state.gyro.dynNotch = _dyn_notch_sma.update(_model.state.gyro.adc);
  }

  if (_dyn_notch_enabled)
  {
    for (size_t p = 0; p < _dyn_notch_count; p++)
    {
      _model.state.gyro.adc = Utils::applyFilter(_model.state.gyro.dynNotchFilter[p], _model.state.gyro.adc);
    }
  }

  for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    _model.setDebug(DEBUG_GYRO_FILTERED, i, lrintf(Utils::toDeg(_model.state.gyro.adc[i])));
  }

  if (_model.accelActive())
  {
    _model.state.attitude.rate = Utils::applyFilter(_model.state.attitude.filter, _model.state.gyro.adc);
  }

  return 1;
}

void FAST_CODE_ATTR GyroSensor::postLoop()
{
  rpmFilterUpdate();
  dynNotchFilterUpdate();
}

void FAST_CODE_ATTR GyroSensor::rpmFilterUpdate()
{
  if (!_rpm_enabled) return;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_RPM_UPDATE);

  const float motorFreq = _model.state.output.telemetry.freq[_rpm_motor_index];
  for (size_t n = 0; n < _model.config.gyro.rpmFilter.harmonics; n++)
  {
    const float freq = Utils::clamp(motorFreq * (n + 1), _rpm_min_freq, _rpm_max_freq);
    const float freqMargin = freq - _rpm_min_freq;
    float weight = _rpm_weights[n];
    if (freqMargin < _model.config.gyro.rpmFilter.fade)
    {
      weight *= freqMargin * _rpm_fade_inv;
    }
    _model.state.gyro.rpmFilter[_rpm_motor_index][n][0].reconfigure(freq, freq, _rpm_q, weight);
    for (size_t i = 1; i < AXIS_COUNT_RPY; ++i)
    {
      // copy coefs from roll to pitch and yaw
      _model.state.gyro.rpmFilter[_rpm_motor_index][n][i].reconfigure(_model.state.gyro.rpmFilter[_rpm_motor_index][n][0]);
    }
  }

  _model.setDebug(DEBUG_RPM_FILTER, _rpm_motor_index, lrintf(_model.state.output.telemetry.freq[_rpm_motor_index]));

  _rpm_motor_index++;
  if (_rpm_motor_index >= RPM_FILTER_MOTOR_MAX)
  {
    _rpm_motor_index = 0;
  }
}

void FAST_CODE_ATTR GyroSensor::dynNotchFilterUpdate()
{
  if (!_model.gyroActive()) return;
  if (_model.state.loopTimer.rate < DynamicFilterConfig::MIN_FREQ) return;

  if (_dyn_notch_enabled || _dyn_notch_debug)
  {
    Utils::Stats::Measure measure(_model.state.stats, COUNTER_GYRO_FFT);

    const float q = _model.config.gyro.dynamicFilter.q * 0.01;
    bool feed = _model.state.loopTimer.iteration % _dyn_notch_denom == 0;
    bool update = _model.state.gyro.dynamicFilterTimer.check();

    for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
    {
#ifdef ESPFC_DSP
      (void)update;
      if (feed)
      {
        uint32_t startTime = micros();
        int status = _fft[i].update(_model.state.gyro.dynNotch[i]);
        if (_model.config.debug.mode == DEBUG_FFT_TIME)
        {
          if (i == 0) _model.state.debug[0] = status;
          _model.state.debug[i + 1] = micros() - startTime;
        }
        if (_model.config.debug.mode == DEBUG_FFT_FREQ && i == _model.config.debug.axis)
        {
          _model.state.debug[0] = lrintf(_fft[i].peaks[0].freq);
          _model.state.debug[1] = lrintf(_fft[i].peaks[1].freq);
          _model.state.debug[2] = lrintf(_fft[i].peaks[2].freq);
          _model.state.debug[3] = lrintf(_fft[i].peaks[3].freq);
        }
        if (_dyn_notch_enabled && status)
        {
          for (size_t p = 0; p < _dyn_notch_count; p++)
          {
            float freq = _fft[i].peaks[p].freq;
            if (freq >= _model.config.gyro.dynamicFilter.min_freq && freq <= _model.config.gyro.dynamicFilter.max_freq)
            {
              _model.state.gyro.dynNotchFilter[p][i].reconfigure(freq, freq, q);
            }
          }
        }
      }
#else
      if (feed)
      {
        uint32_t startTime = micros();
        _freqAnalyzer[i].update(_model.state.gyro.dynNotch[i]);
        float freq = _freqAnalyzer[i].freq;
        if (_model.config.debug.mode == DEBUG_FFT_TIME)
        {
          if (i == 0) _model.state.debug[0] = update;
          _model.state.debug[i + 1] = micros() - startTime;
        }
        if (_model.config.debug.mode == DEBUG_FFT_FREQ)
        {
          if (update) _model.state.debug[i] = lrintf(freq);
          if (i == _model.config.debug.axis) _model.state.debug[3] = lrintf(Utils::toDeg(_model.state.gyro.dynNotch[i]));
        }
        if (_dyn_notch_enabled && update)
        {
          if (freq >= _model.config.gyro.dynamicFilter.min_freq && freq <= _model.config.gyro.dynamicFilter.max_freq)
          {
            for (size_t p = 0; p < _dyn_notch_count; p++)
            {
              size_t x = (p + i) % 3;
              int harmonic = (p / 3) + 1;
              int16_t f = Utils::clamp((int16_t)lrintf(freq * harmonic), _model.config.gyro.dynamicFilter.min_freq, _model.config.gyro.dynamicFilter.max_freq);
              _model.state.gyro.dynNotchFilter[p][x].reconfigure(f, f, q);
            }
          }
        }
      }
#endif
    }
  }
}

void FAST_CODE_ATTR GyroSensor::calibrate()
{
  switch (_model.state.gyro.calibrationState)
  {
  case CALIBRATION_IDLE:
    _model.state.gyro.adc -= _model.state.gyro.bias;
    break;
  case CALIBRATION_START:
    //_model.state.gyro.bias = VectorFloat();
    _model.state.gyro.biasSamples = 2 * _model.state.gyro.calibrationRate;
    _model.state.gyro.calibrationState = CALIBRATION_UPDATE;
    break;
  case CALIBRATION_UPDATE:
  {
    VectorFloat deltaAccel = _model.state.accel.adc - _model.state.accel.prev;
    _model.state.accel.prev = _model.state.accel.adc;
    if (deltaAccel.getMagnitude() < ESPFC_FUZZY_ACCEL_ZERO && _model.state.gyro.adc.getMagnitude() < ESPFC_FUZZY_GYRO_ZERO)
    {
      _model.state.gyro.bias += (_model.state.gyro.adc - _model.state.gyro.bias) * _model.state.gyro.biasAlpha;
      _model.state.gyro.biasSamples--;
    }
    if (_model.state.gyro.biasSamples <= 0)
      _model.state.gyro.calibrationState = CALIBRATION_APPLY;
  }
  break;
  case CALIBRATION_APPLY:
    _model.state.gyro.calibrationState = CALIBRATION_SAVE;
    break;
  case CALIBRATION_SAVE:
    _model.finishCalibration();
    _model.state.gyro.calibrationState = CALIBRATION_IDLE;
    break;
  default:
    _model.state.gyro.calibrationState = CALIBRATION_IDLE;
    break;
  }
}

}
