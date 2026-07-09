#include "Control/Fusion.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Control {

Fusion::Fusion(Model& model): _model(model), _madgwick(), _mahony(), _rtqf(), _useMag(false) {}

int Fusion::begin()
{
  _useMag = _model.config.fusion.useMag;

  _madgwick.begin(_model.state.accel.timer.rate);
  _madgwick.setKp(_model.config.fusion.gain * 0.05f);

  _mahony.begin(_model.state.accel.timer.rate);
  _mahony.setKp(_model.config.fusion.gain * 0.1f);
  _mahony.setKi(_model.config.fusion.gainI * 0.1f);

  _rtqf.begin(_model.state.accel.timer.rate);
  _rtqf.setKp(_model.config.fusion.gain * 0.0002f);

  _model.logger.info()
      .log(F("FUSION"))
      .log(FPSTR(FusionConfig::getModeName((FusionMode)_model.config.fusion.mode)))
      .logln(_model.config.fusion.gain);

  for (size_t i = 0; i < 4; i++)
  {
    _qFilter[i].begin(FilterConfig(FILTER_BIQUAD, 20), _model.state.accel.timer.rate);
  }

  return 1;
}

void Fusion::restoreGain()
{
  _madgwick.setKp(_model.config.fusion.gain * 0.005f);
  _mahony.setKp(_model.config.fusion.gain * 0.02f);
  _mahony.setKi(_model.config.fusion.gainI * 0.02f);
  _rtqf.setKp(_model.config.fusion.gain * 0.00005f);
}

int FAST_CODE_ATTR Fusion::update()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_IMU_FUSION);

  if (_model.accelActive())
  {
    const auto& g = _model.state.attitude.rate;
    const auto a = _model.state.accel.adc.fetch();
    const auto& m = _model.state.mag.adc;
    Quaternion q;

    switch (_model.config.fusion.mode)
    {
      case FUSION_MADGWICK: q = madgwickFusion(g, a, m); break;
      case FUSION_MAHONY: q = mahonyFusion(g, a, m); break;
      case FUSION_RTQF: q = rtqfFusion(g, a, m); break;
      case FUSION_NONE:
      default: break;
    }

    _model.state.attitude.quaternion = Quaternion::ensureSign(q, _model.state.attitude.quaternion);
    _model.state.attitude.euler.eulerFromQuaternion(_model.state.attitude.quaternion);

    // filter quaternion to align delay with accelerometer filtering
    const auto fq = filterQuaternion(_model.state.attitude.quaternion).getNormalized();

    auto world = a.getRotated(fq);
    world.z -= ACCEL_G; // remove gravity

    _model.state.accel.world = world;
    _model.state.attitude.cosTheta = 1.0f - 2.0f * (fq.x * fq.x + fq.y * fq.y);
  }

  if (_model.config.debug.mode == DEBUG_AC_ERROR)
  {
    _model.state.debug[0] = lrintf(_model.state.accel.world[0] * ACCEL_G_INV * 1000);
    _model.state.debug[1] = lrintf(_model.state.accel.world[1] * ACCEL_G_INV * 1000);
    _model.state.debug[2] = lrintf(_model.state.accel.world[2] * ACCEL_G_INV * 1000);
    _model.state.debug[3] = lrintf(_model.state.attitude.cosTheta * 1000.f);
  }

  if (_model.config.debug.mode == DEBUG_AC_CORRECTION)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(_model.state.attitude.euler[0]) * 10);
    _model.state.debug[1] = lrintf(Utils::toDeg(_model.state.attitude.euler[1]) * 10);
    _model.state.debug[2] = lrintf(Utils::toDeg(_model.state.attitude.euler[2]) * 10);
  }
  return 1;
}

Quaternion Fusion::filterQuaternion(const Quaternion& q)
{
  return {_qFilter[0].update(q.w), _qFilter[1].update(q.x), _qFilter[2].update(q.y), _qFilter[3].update(q.z)};
}

Quaternion FAST_CODE_ATTR Fusion::madgwickFusion(VectorFloat g, VectorFloat a, VectorFloat m)
{
  if (_useMag && _model.magActive())
  {
    _madgwick.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
  }
  else
  {
    _madgwick.update(g.x, g.y, g.z, a.x, a.y, a.z);
  }
  return _madgwick.getQuaternion();
}

Quaternion FAST_CODE_ATTR Fusion::mahonyFusion(VectorFloat g, VectorFloat a, VectorFloat m)
{
  if (_useMag && _model.magActive())
  {
    _mahony.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
  }
  else
  {
    _mahony.update(g.x, g.y, g.z, a.x, a.y, a.z);
  }
  return _mahony.getQuaternion();
}

Quaternion FAST_CODE_ATTR Fusion::rtqfFusion(VectorFloat g, VectorFloat a, VectorFloat m)
{
  if (_useMag && _model.magActive())
  {
    _rtqf.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
  }
  else
  {
    _rtqf.update(g.x, g.y, g.z, a.x, a.y, a.z);
  }
  return _rtqf.getQuaternion();
}

} // namespace Control

} // namespace Espfc
