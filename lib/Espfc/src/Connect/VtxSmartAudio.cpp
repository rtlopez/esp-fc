#include "VtxSmartAudio.hpp"
#include "Utils/Crc.hpp"

static const uint8_t dummyByte[] = { 0x00 };

namespace Espfc::Connect {

int VtxSmartAudio::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  _timer.setRate(300);

  _state = State::INIT;
  return 1;
}

int VtxSmartAudio::update()
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

int VtxSmartAudio::setChannel()
{
  uint8_t vtxCommand[6] = { 0xAA, 0x55, 0x07, 0x01, (uint8_t)((_model.config.vtx.band -1)*8 + _model.config.vtx.channel - 1) };
  vtxCommand[5] = Utils::crc8_dvb_s2(vtxCommand[5], reinterpret_cast<uint8_t*>(&vtxCommand), 5);
  _serial->write(dummyByte, 1);
  _serial->write(vtxCommand, 6);
  _serial->flush();
  return 1;
}

int VtxSmartAudio::setPower()
{
  uint8_t vtxCommand[6] = { 0xAA, 0x55, 0x05, 0x01, (uint8_t)((!_model.config.vtx.lowPowerDisarm || _model.isModeActive(MODE_ARMED)) ? _model.config.vtx.power - 1 : 0) };
  vtxCommand[5] = Utils::crc8_dvb_s2(vtxCommand[5], reinterpret_cast<uint8_t*>(&vtxCommand), 5);
  _serial->write(dummyByte, 1);
  _serial->write(vtxCommand, 6);
  _serial->flush();
  return 1;
}

}
