#include "VtxTramp.hpp"
#include "Utils/Crc.hpp"

namespace Espfc::Connect {

int VtxTramp::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  _timer.setInterval(300);

  _state = State::INIT;
  return 1;
}

int VtxTramp::update()
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

int VtxTramp::trampSendCommand(uint8_t cmd, uint16_t param)
{
    uint8_t vtxCommand[16] = {0}; // fully zeroed like Betaflight
    vtxCommand[0] = 0x0F;         // sync
    vtxCommand[1] = cmd;          // command
    vtxCommand[2] = param & 0xFF;
    vtxCommand[3] = (param >> 8) & 0xFF;

    // bytes 4..13 already zeroed
    vtxCommand[14] = Utils::crc8_dvb_s2(0, vtxCommand, 14); // CRC over bytes 0–13
    vtxCommand[15] = 0x00; // end byte

    _serial->write(vtxCommand, 16);
    _serial->flush();
    return 1;
}

int VtxTramp::setChannel()
{
  trampSendCommand('F', TrampFreqTable[_model.config.vtx.band -1][_model.config.vtx.channel -1]);
  return 1;
}

int VtxTramp::setPower()
{
  uint16_t power = (!_model.config.vtx.lowPowerDisarm/* || _model.isModeActive(MODE_ARMED)*/) ? 
      TrampPowerTable[_model.config.vtx.power - 1] : 0;
  trampSendCommand('P', power);
  return 1;
}

int VtxTramp::setPitMode()
{
  trampSendCommand('I', _model.config.vtx.pitMode ? 0 : 1);
  return 1;
}

}
