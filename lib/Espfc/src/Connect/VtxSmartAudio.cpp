#include "VtxSmartAudio.hpp"
#include "Hal/Time.hpp"
#include "Utils/Crc.hpp"
#include <algorithm>

namespace Espfc::Connect {

namespace {

constexpr uint32_t REFRESH_US = 400000;
constexpr uint8_t SYNC_BYTE = 0x00;
constexpr uint8_t CMD_SET_POWER = 0x05;
constexpr uint8_t CMD_SET_CHANNEL = 0x07;

void encode(uint8_t* frame, uint8_t cmd, uint8_t value)
{
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = cmd;
  frame[3] = 0x01;
  frame[4] = value;
  frame[5] = Utils::crc8_dvb_s2(0, frame, 5);
}

} // namespace

int VtxSmartAudio::begin(Stream::ReadWritable* serial)
{
  _serial = serial;
  _next = Command::CHANNEL;
  _timer.setInterval(REFRESH_US / static_cast<uint32_t>(Command::COUNT));
  _model.state.vtx.type = VTXDEV_SMARTAUDIO;
  _model.state.vtx.active = serial != nullptr;
  return 1;
}

int VtxSmartAudio::update()
{
  return update(micros());
}

int VtxSmartAudio::update(uint32_t now)
{
  if (!_serial || !_timer.check(now)) return 1;

  switch (_next)
  {
    case Command::CHANNEL:
      sendChannel();
      _next = Command::POWER;
      break;
    default:
      sendPower();
      _next = Command::CHANNEL;
      break;
  }
  return 1;
}

void VtxSmartAudio::encodeChannel(uint8_t* frame, uint8_t band, uint8_t channel)
{
  const uint8_t b = std::clamp<uint8_t>(band, 1, BANDS) - 1;
  const uint8_t c = std::clamp<uint8_t>(channel, 1, CHANNELS) - 1;
  encode(frame, CMD_SET_CHANNEL, b * CHANNELS + c);
}

void VtxSmartAudio::encodePower(uint8_t* frame, uint8_t power)
{
  encode(frame, CMD_SET_POWER, power > 0 ? power - 1 : 0);
}

void VtxSmartAudio::send(const uint8_t* frame)
{
  if (_serial->availableForWrite() < static_cast<int>(FRAME_SIZE + 1)) return;
  _serial->write(SYNC_BYTE);
  _serial->write(frame, FRAME_SIZE);
}

void VtxSmartAudio::sendChannel()
{
  uint8_t frame[FRAME_SIZE];
  encodeChannel(frame, _model.config.vtx.band, _model.config.vtx.channel);
  send(frame);
}

void VtxSmartAudio::sendPower()
{
  const auto& c = _model.config.vtx;
  const bool low = c.lowPowerDisarm && !_model.isModeActive(MODE_ARMED);
  uint8_t frame[FRAME_SIZE];
  encodePower(frame, low ? 1 : c.power);
  send(frame);
}

} // namespace Espfc::Connect
