#include "VoltageSensor.hpp"

#include "Hal/Adc.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Sensor {

VoltageSensor::VoltageSensor(Model& model): _model(model) {}

int VoltageSensor::begin()
{
  _model.state.battery.timer.setRate(100);
  _model.state.battery.samples = 50;

#ifdef ESPFC_ADC_VBAT
  if (_model.config.pin[PIN_INPUT_ADC_VBAT] != -1) Hal::Adc::begin(_model.config.pin[PIN_INPUT_ADC_VBAT]);
#endif
#ifdef ESPFC_ADC_IBAT
  if (_model.config.pin[PIN_INPUT_ADC_IBAT] != -1) Hal::Adc::begin(_model.config.pin[PIN_INPUT_ADC_IBAT]);
#endif

  reload(MODEL_CHANGE_FILTER);

  _state = VBAT;

  return 1;
}

int VoltageSensor::reload(ModelChangeEvent event)
{
  switch (event)
  {
    case MODEL_CHANGE_FILTER:
      _vFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _vFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);
      _iFilterFast.begin(FilterConfig(FILTER_PT1, 20), _model.state.battery.timer.rate);
      _iFilter.begin(FilterConfig(FILTER_PT2, 2), _model.state.battery.timer.rate);
      break;
    case MODEL_CHANGE_ADC:
      if (_model.config.vbat.source != VBAT_SOURCE_ADC)
      {
        _model.state.battery.rawVoltage = 0;
        _model.state.battery.voltageUnfiltered = 0.0f;
        _model.state.battery.voltage = 0.0f;
        _model.state.battery.cellVoltage = 0.0f;
        _model.state.battery.percentage = 0.0f;
        _model.state.battery.cells = 0;
      }
      else
      {
        _model.state.battery.samples = 50; // redetect cell count
      }
      if (_model.config.ibat.source != IBAT_SOURCE_ADC)
      {
        _model.state.battery.rawCurrent = 0;
        _model.state.battery.currentUnfiltered = 0.0f;
        _model.state.battery.current = 0.0f;
      }
      break;
    default:
      break;
  }
  return 1;
}

int VoltageSensor::update()
{
  if (!_model.state.battery.timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_BATTERY);

  switch (_state)
  {
    case VBAT:
      _state = IBAT;
      return readVbat();
    case IBAT:
      _state = VBAT;
      return readIbat();
  }

  return 0;
}

int VoltageSensor::readVbat()
{
#ifdef ESPFC_ADC_VBAT
  if (_model.config.vbat.source != VBAT_SOURCE_ADC || _model.config.pin[PIN_INPUT_ADC_VBAT] == -1) return 0;
  // wemos d1 mini has divider 3.2:1 (220k:100k)
  // additionaly I've used divider 5.7:1 (4k7:1k)
  // total should equals ~18.24:1, 73:4 resDiv:resMult should be ideal,
  // but ~52:1 is real, did I miss something?
  _model.state.battery.rawVoltage = Hal::Adc::read(_model.config.pin[PIN_INPUT_ADC_VBAT]);
  float volts = _vFilterFast.update(_model.state.battery.rawVoltage * ESPFC_ADC_SCALE);

  volts *= _model.config.vbat.scale * 0.1f;
  volts *= _model.config.vbat.resMult;
  volts /= _model.config.vbat.resDiv;

  _model.state.battery.voltageUnfiltered = volts;
  _model.state.battery.voltage = _vFilter.update(_model.state.battery.voltageUnfiltered);

  // cell count detection
  if (_model.state.battery.samples > 0)
  {
    _model.state.battery.cells = std::ceil(_model.state.battery.voltage / 4.2f);
    _model.state.battery.samples--;
  }

  _model.state.battery.cellVoltage = _model.state.battery.voltage / std::clamp<int>(_model.state.battery.cells, 1, 6);
  _model.state.battery.percentage =
      std::clamp(Utils::map(_model.state.battery.cellVoltage, 3.4f, 4.2f, 0.0f, 100.0f), 0.0f, 100.0f);

  if (_model.config.debug.mode == DEBUG_BATTERY)
  {
    _model.state.debug[0] = std::clamp<long>(lrintf(_model.state.battery.voltageUnfiltered * 100.0f), 0L, 32000L);
    _model.state.debug[1] = std::clamp<long>(lrintf(_model.state.battery.voltage * 100.0f), 0L, 32000L);
  }
  return 1;
#else
  return 0;
#endif
}

int VoltageSensor::readIbat()
{
#ifdef ESPFC_ADC_IBAT
  if (_model.config.ibat.source != IBAT_SOURCE_ADC || _model.config.pin[PIN_INPUT_ADC_IBAT] == -1) return 0;

  _model.state.battery.rawCurrent = Hal::Adc::read(_model.config.pin[PIN_INPUT_ADC_IBAT]);
  float volts = _iFilterFast.update(_model.state.battery.rawCurrent * ESPFC_ADC_SCALE);
  float milivolts = volts * 1000.0f;

  volts += _model.config.ibat.offset * 0.001f;
  volts *= _model.config.ibat.scale * 0.1f;

  _model.state.battery.currentUnfiltered = volts;
  _model.state.battery.current = _iFilter.update(_model.state.battery.currentUnfiltered);

  if (_model.config.debug.mode == DEBUG_CURRENT_SENSOR)
  {
    _model.state.debug[0] = lrintf(milivolts);
    _model.state.debug[1] = std::clamp<long>(lrintf(_model.state.battery.currentUnfiltered * 100.0f), 0L, 32000L);
    _model.state.debug[2] = _model.state.battery.rawCurrent;
  }

  return 1;
#else
  return 0;
#endif
}

} // namespace Espfc::Sensor
