#include "Vtx.hpp"
#include "Utils/Crc.hpp"

static const uint8_t dummyByte[] = {0x00};

namespace Espfc::Connect {

int Vtx::begin(Device::SerialDevice* serial)
{
  _serial = serial;
  _timer.setRate(300000);

  _state = State::INIT;
  return 1;
}

int Vtx::update()
{
  if (!_timer.check()) return 1;
  switch (_state)
  {
    case State::INIT:
      _state = State::SET_CHANNEL;
      _model.state.vtx.active = true;
      break;
    case State::SET_POWER:
      setPower();
      _state = State::IDLE;
      break;
    case State::SET_CHANNEL:
      setChannel();
      _state = State::SET_POWER;
      break;
    case State::IDLE:
      if (_model.isModeActive(MODE_ARMED) != _armed)
      {
        _armed = !_armed;
        _state = State::SET_POWER;
      }
      break;
    case State::INACTIVE:
    default:
      break;
  }

  return 1;
}

int Vtx::setChannel()
{
  uint8_t band = _model.config.vtx.band - 1;
  uint8_t channel = _model.config.vtx.channel - 1;
  uint8_t code = static_cast<uint8_t>(band * 8 + channel);
  uint8_t vtxCommand[6] = {0xAA, 0x55, 0x07, 0x01, code};
  vtxCommand[5] = Utils::crc8_dvb_s2(0, vtxCommand, 5);
  _serial->write(dummyByte, 1);
  _serial->write(vtxCommand, 6);
  _serial->flush();

  return 1;
}

int Vtx::setPower()
{
  uint8_t power = _model.config.vtx.power - 1;
  bool armed = _model.isModeActive(MODE_ARMED);
  bool lowPowerDisarm = _model.config.vtx.lowPowerDisarm;
  uint8_t code = static_cast<uint8_t>((!lowPowerDisarm || armed) ? power : 0);
  uint8_t vtxCommand[6] = {0xAA, 0x55, 0x05, 0x01, code};
  vtxCommand[5] = Utils::crc8_dvb_s2(0, vtxCommand, 5);
  _serial->write(dummyByte, 1);
  _serial->write(vtxCommand, 6);
  _serial->flush();

  return 1;
}

} // namespace Espfc::Connect
