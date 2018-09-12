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

      _model.logger.info().log(F("MAG INIT")).log(FPSTR(Device::MagDevice::getName(_mag->getType()))).logln(_model.state.magTimer.rate);

      return 1;
    }

    int update()
    {
      if(!_model.magActive() || !_model.state.magTimer.check()) return 0;
      if(!_mag) return 0;
      //return 1;

      {
        Stats::Measure measure(_model.state.stats, COUNTER_MAG_READ);
        _mag->readMag(_model.state.magRaw);
      }

      {      
        Stats::Measure measure(_model.state.stats, COUNTER_MAG_FILTER);

        align(_model.state.magRaw, _model.config.magAlign);
        _model.state.mag = _mag->convert(_model.state.magRaw);
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