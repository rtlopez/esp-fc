#ifndef _ESPFC_SENSOR_MAG_SENSOR_H_
#define _ESPFC_SENSOR_MAG_SENSOR_H_

#include "BaseSensor.h"
#include "Device/MagDevice.h"

#include <math.h>

namespace Espfc {

namespace Sensor {

class MagSensor: public BaseSensor
{
  public:
    MagSensor(Model& model): _model(model) {}

    int begin()
    {
      if(!_model.magActive()) return 0;

      _mag = Hardware::getMagDevice(_model);
      if(!_mag) return 0;

      _mag->setSampleAveraging(_model.config.magAvr);
      _mag->setMode(HMC5883L_MODE_CONTINUOUS);

      int rate = -1;
      switch(_model.config.magSampleRate)
      {
        case MAG_RATE_3:    rate = 3; break;
        case MAG_RATE_7P5:  rate = 7; break;
        case MAG_RATE_15:   rate = 15; break;
        case MAG_RATE_30:   rate = 30; break;
        case MAG_RATE_75:   rate = 75; break;
        default: _model.config.magSampleRate = MAG_RATE_15; rate = 15;
      }
      _mag->setSampleRate(_model.config.magSampleRate + 0x02);
      _model.state.magTimer.setRate(rate);

      const float base = 1.0f; // 1000.0;
      switch(_model.config.magFsr)
      {
        case MAG_GAIN_1370: _model.state.magScale = base / 1370; break;
        case MAG_GAIN_1090: _model.state.magScale = base / 1090; break;
        case MAG_GAIN_820:  _model.state.magScale = base / 820; break;
        case MAG_GAIN_660:  _model.state.magScale = base / 660; break;
        case MAG_GAIN_440:  _model.state.magScale = base / 440; break;
        case MAG_GAIN_390:  _model.state.magScale = base / 390; break;
        case MAG_GAIN_330:  _model.state.magScale = base / 330; break;
        case MAG_GAIN_220:  _model.state.magScale = base / 220; break;
      }
      _mag->setGain(_model.config.magFsr);

      _model.logger.info().log(F("MAG RATE")).log(_model.config.magSampleRate).log(_model.state.magTimer.rate).logln(_model.state.magTimer.interval);
      _model.logger.info().log(F("MAG INIT")).logln(_model.state.magPresent);

      return 1;
    }

    int update()
    {
      if(!_model.magActive() || !_model.state.magTimer.check()) return 0;
      if(!_mag) return 0;

      {
        Stats::Measure measure(_model.state.stats, COUNTER_MAG_READ);
        _mag->readMag(_model.state.magRaw);
      }

      {      
        Stats::Measure measure(_model.state.stats, COUNTER_MAG_FILTER);

        align(_model.state.magRaw, _model.config.magAlign);
        _model.state.mag  = (VectorFloat)_model.state.magRaw  * _model.state.magScale;
        for(size_t i = 0; i < 3; i++)
        {
          _model.state.mag.set(i, _model.state.magFilter[i].update(_model.state.mag[i]));
        }
        if(_model.state.magCalibrationValid && _model.state.magCalibration == 0)
        {
          _model.state.mag -= _model.state.magCalibrationOffset;
          _model.state.mag *= _model.state.magCalibrationScale;
        }
      }
      collectMagCalibration();

      return 1;
    }

    void collectMagCalibration()
    {
      if(!_model.magActive()) return;
      if(_model.state.magCalibration == 1)
      {
        resetMagCalibration();
        _model.state.magCalibration = 2;
      }

      if(_model.state.magCalibration == 0) return;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationData[i][0] = _model.state.mag.get(i) < _model.state.magCalibrationData[i][0] ? _model.state.mag.get(i) : _model.state.magCalibrationData[i][0];
        _model.state.magCalibrationData[i][1] = _model.state.mag.get(i) > _model.state.magCalibrationData[i][1] ? _model.state.mag.get(i) : _model.state.magCalibrationData[i][1];
      }
      updateMagCalibration();
    }

    void resetMagCalibration()
    {
      if(!_model.magActive()) return;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationData[i][0] = 0;
        _model.state.magCalibrationData[i][1] = 0;
      }
      updateMagCalibration();
    }

    void updateMagCalibration()
    {
      if(!_model.magActive()) return;
      // just in case when the calibration data is not valid
      _model.state.magCalibrationValid = false;
      for(int i = 0; i < 3; i++)
      {
        _model.state.magCalibrationOffset.set(i, 0.f);
        _model.state.magCalibrationScale.set(i, 1.f);
      }

      // find biggest range
      float maxDelta = -1;
      for(int i = 0; i < 3; i++)
      {
        if((_model.state.magCalibrationData[i][1] - _model.state.magCalibrationData[i][0]) > maxDelta)
        {
          maxDelta = _model.state.magCalibrationData[i][1] - _model.state.magCalibrationData[i][0];
        }
      }

      if(maxDelta <= 0) return;
      const float epsilon = 0.001f;
      maxDelta /= 2;                                         // this is the max +/- range
      for (int i = 0; i < 3; i++)
      {
        float delta = (_model.state.magCalibrationData[i][1] - _model.state.magCalibrationData[i][0]) / 2.f;
        if(delta < epsilon && delta > -epsilon) return;
        float offset = (_model.state.magCalibrationData[i][1] + _model.state.magCalibrationData[i][0]) / 2.f;
        _model.state.magCalibrationScale.set(i, maxDelta / delta);     // makes everything the same range
        _model.state.magCalibrationOffset.set(i, offset);
      }
      _model.state.magCalibrationValid = true;
    }

  private:
    Model& _model;
    Device::MagDevice * _mag;
};

}

}
#endif