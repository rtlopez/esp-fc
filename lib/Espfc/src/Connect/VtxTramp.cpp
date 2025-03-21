#include "VtxTramp.hpp"
#include "Utils/Crc.hpp"

static const uint8_t dummyByte[] = { 0x00 };

namespace Espfc::Connect {

int VtxTramp::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  _timer.setRate(300);

  _state = State::INIT;
  return 1;
}

int VtxTramp::initTramp() 
{
  // Send initialization command
  TrampCommand initCmd;
  initCmd.command = 'r'; // 'r' for reset/init
  initCmd.crc = Utils::crc8_dvb_s2(0, reinterpret_cast<uint8_t*>(&initCmd), sizeof(initCmd) - 2);
  _serial->write(reinterpret_cast<uint8_t*>(&initCmd), sizeof(initCmd));
  _serial->flush();
  return 1;
}

int VtxTramp::update()
{
  if (!_timer.check()) return 1;
  switch (_state)
  {
    case State::INIT:
      initTramp();  
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

int VtxTramp::setChannel()
{
  uint8_t vtxCommand[6];
  vtxCommand[0] = 0x0F;
  vtxCommand[1] = 0x55;
  vtxCommand[2] = 0x00;
  vtxCommand[3] = 0x00;
  vtxCommand[4] = (_model.config.vtx.band - 1) * 8 + (_model.config.vtx.channel - 1);
  vtxCommand[5] = Utils::crc8_dvb_s2(0, reinterpret_cast<uint8_t*>(&vtxCommand), 5);
  _serial->write(vtxCommand, 6);
  _serial->flush();
  return 1;
}

int VtxTramp::setPower()
{
  uint8_t vtxCommand[6];
  vtxCommand[0] = 0x0F;
  vtxCommand[1] = 0x56;
  vtxCommand[2] = 0x00;
  vtxCommand[3] = 0x00;
  vtxCommand[4] = (!_model.config.vtx.lowPowerDisarm || _model.isModeActive(MODE_ARMED)) ? _model.config.vtx.power : 0;
  vtxCommand[5] = Utils::crc8_dvb_s2(0, reinterpret_cast<uint8_t*>(&vtxCommand), 5);
  _serial->write(vtxCommand, 6);
  _serial->flush();
  return 1;
}

}
