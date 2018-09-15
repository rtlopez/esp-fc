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
    enum CalibrationState {
      MAG_CALIBRATION_IDLE   = 0,
      MAG_CALIBRATION_RESET  = 1,
      MAG_CALIBRATION_UPDATE = 2,
      MAG_CALIBRATION_APPLY  = 3,
    };

    MagSensor(Model& model): _model(model) {}

    int begin()
    {
      if(!_model.magActive()) return 0;

      _mag = Hardware::getMagDevice(_model);
      if(!_mag) return 0;

      _model.logger.info().log(F("MAG INIT")).log(FPSTR(Device::MagDevice::getName(_mag->getType()))).logln(_model.state.magTimer.rate);
      _model.state.magCalibrationValid = true;

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
        _model.state.mag = _mag->convert(_model.state.magRaw);
        if(_model.state.magCalibrationValid && _model.state.magCalibrationState == MAG_CALIBRATION_IDLE)
        {
          _model.state.mag -= _model.state.magCalibrationOffset;
          _model.state.mag *= _model.state.magCalibrationScale;
        }
        for(size_t i = 0; i < 3; i++)
        {
          _model.state.mag.set(i, _model.state.magFilter[i].update(_model.state.mag[i]));
        }
      }
      magCalibration();

      return 1;
    }

    void magCalibration()
    {
      switch(_model.state.magCalibrationState)
      {
        case MAG_CALIBRATION_RESET:
          resetMagCalibration();
          _model.state.magCalibrationSamples = 30 * _model.state.magTimer.rate;
          _model.state.magCalibrationState = MAG_CALIBRATION_UPDATE;
          break;
        case MAG_CALIBRATION_UPDATE:
          updateMagCalibration();
          _model.state.magCalibrationSamples--;
          if(_model.state.magCalibrationSamples <= 0) _model.state.magCalibrationState = MAG_CALIBRATION_APPLY;
          break;
        case MAG_CALIBRATION_APPLY:
          applyMagCalibration();
          _model.state.magCalibrationState = MAG_CALIBRATION_IDLE;
          break;
        case MAG_CALIBRATION_IDLE:
        default:
          return;
      }
    }

    void resetMagCalibration()
    {
      _model.state.magCalibrationMin = VectorFloat();
      _model.state.magCalibrationMax = VectorFloat();
      _model.state.magCalibrationOffset = VectorFloat();
      _model.state.magCalibrationScale = VectorFloat(1.f, 1.f, 1.f);
      _model.state.magCalibrationValid = false;
    }

    void updateMagCalibration()
    {
      for(int i = 0; i < 3; i++)
      {
        if(_model.state.mag[i] < _model.state.magCalibrationMin[i]) _model.state.magCalibrationMin.set(i, _model.state.mag[i]);
        if(_model.state.mag[i] > _model.state.magCalibrationMax[i]) _model.state.magCalibrationMax.set(i, _model.state.mag[i]);
      }
    }

    void applyMagCalibration()
    {
      // verify calibration data and find biggest range
      float maxDelta = -1;
      for(int i = 0; i < 3; i++)
      {
        if(_model.state.magCalibrationMin[i] > -0.01f) return;
        if(_model.state.magCalibrationMax[i] <  0.01f) return;
        if((_model.state.magCalibrationMax[i] - _model.state.magCalibrationMin[i]) > maxDelta)
        {
          maxDelta = _model.state.magCalibrationMax[i] - _model.state.magCalibrationMin[i];
        }
      }

      // probably incomplete data, must be positive
      if(maxDelta <= 0) return;

      const float epsilon = 0.001f;
      maxDelta /= 2;                                                   // this is the max +/- range
      for (int i = 0; i < 3; i++)
      {
        float delta = (_model.state.magCalibrationMax[i] - _model.state.magCalibrationMin[i]) * 0.5f;
        if(abs(delta) < epsilon) return;
        _model.state.magCalibrationScale.set(i, maxDelta / delta);     // makes everything the same range

        float offset = (_model.state.magCalibrationMax[i] + _model.state.magCalibrationMin[i]) * 0.5f;
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