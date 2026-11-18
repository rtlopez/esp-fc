#include "VtxSmartAudio.hpp"
#include "Hal/Time.hpp"
#include "Utils/Crc.hpp"
#include "Vtx.hpp"
#include <algorithm>

namespace Espfc::Connect {

namespace {

constexpr uint32_t REFRESH_US = 400000;
constexpr uint8_t SYNC_BYTE = 0x00;
constexpr uint8_t CMD_SET_POWER = 0x05;
constexpr uint8_t CMD_SET_CHANNEL = 0x07;
constexpr uint8_t CMD_SET_FREQUENCY = 0x09;
constexpr uint8_t CMD_SET_MODE = 0x0B;

constexpr uint8_t MODE_CLEAR_PIT_MODE = 0x04;
constexpr uint8_t MODE_UNLOCK = 0x08;
constexpr uint8_t POWER_DBM_FLAG = 0x80;
constexpr uint16_t FREQUENCY_SET_PIT_FLAG = 0x8000;

constexpr uint8_t POWER_DAC_V1[VtxSmartAudio::POWERS] = {7, 16, 25, 40};   // 25, 200, 500, 800 mW
constexpr uint8_t POWER_DBM_V21[VtxSmartAudio::POWERS] = {14, 23, 27, 29}; // 25, 200, 500, 800 mW

size_t encode(uint8_t* frame, uint8_t cmd, const uint8_t* payload, uint8_t len)
{
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = cmd;
  frame[3] = len;
  std::copy(payload, payload + len, frame + 4);
  frame[4 + len] = Utils::crc8_dvb_s2(0, frame, 4 + len);
  return 5 + len;
}

size_t encode(uint8_t* frame, uint8_t cmd, uint8_t value)
{
  return encode(frame, cmd, &value, 1);
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
    case Command::POWER:
      sendPower();
      _next = Command::MODE;
      break;
    case Command::MODE:
      sendMode();
      _next = Command::PIT_FREQUENCY;
      break;
    default:
      sendPitModeFrequency();
      _next = Command::CHANNEL;
      break;
  }
  return 1;
}

size_t VtxSmartAudio::encodeChannel(uint8_t* frame, uint8_t band, uint8_t channel)
{
  const uint8_t b = std::clamp<uint8_t>(band, 1, BANDS) - 1;
  const uint8_t c = std::clamp<uint8_t>(channel, 1, CHANNELS) - 1;
  return encode(frame, CMD_SET_CHANNEL, b * CHANNELS + c);
}

size_t VtxSmartAudio::encodePower(uint8_t* frame, uint8_t power, int8_t version)
{
  const uint8_t index = std::clamp<uint8_t>(power, 1, POWERS) - 1;
  switch (version)
  {
    case VTX_SMARTAUDIO_V1:
      return encode(frame, CMD_SET_POWER, POWER_DAC_V1[index]);
    case VTX_SMARTAUDIO_V21:
      return encode(frame, CMD_SET_POWER, POWER_DBM_V21[index] | POWER_DBM_FLAG);
    default:
      return encode(frame, CMD_SET_POWER, index);
  }
}

size_t VtxSmartAudio::encodePitModePower(uint8_t* frame)
{
  return encode(frame, CMD_SET_POWER, 0 | POWER_DBM_FLAG);
}

size_t VtxSmartAudio::encodeClearPitMode(uint8_t* frame)
{
  return encode(frame, CMD_SET_MODE, MODE_CLEAR_PIT_MODE | MODE_UNLOCK);
}

size_t VtxSmartAudio::encodeFrequency(uint8_t* frame, uint16_t frequency)
{
  const uint8_t payload[2] = {static_cast<uint8_t>(frequency >> 8), static_cast<uint8_t>(frequency & 0xFF)};
  return encode(frame, CMD_SET_FREQUENCY, payload, 2);
}

size_t VtxSmartAudio::encodePitModeFrequency(uint8_t* frame, uint16_t frequency)
{
  return encodeFrequency(frame, frequency | FREQUENCY_SET_PIT_FLAG);
}

void VtxSmartAudio::send(const uint8_t* frame, size_t len)
{
  if (_serial->availableForWrite() < static_cast<int>(len + 1)) return;
  _serial->write(SYNC_BYTE);
  _serial->write(frame, len);
}

void VtxSmartAudio::sendChannel()
{
  const auto& c = _model.config.vtx;
  const uint16_t f = c.smartAudioVersion == VTX_SMARTAUDIO_V1 ? vtxDirectFrequency(c) : vtxBaseFrequency(c);
  if (!f && !c.band) return;
  uint8_t frame[FRAME_SIZE] = {};
  send(frame, f ? encodeFrequency(frame, f) : encodeChannel(frame, c.band, c.channel));
}

void VtxSmartAudio::sendPower()
{
  const auto& c = _model.config.vtx;
  uint8_t frame[FRAME_SIZE] = {};
  if (c.pitMode && c.smartAudioVersion == VTX_SMARTAUDIO_V21)
  {
    send(frame, encodePitModePower(frame));
    return;
  }
  const bool low = c.lowPowerDisarm && !_model.isModeActive(MODE_ARMED);
  send(frame, encodePower(frame, low ? 1 : c.power, c.smartAudioVersion));
}

void VtxSmartAudio::sendMode()
{
  const auto& c = _model.config.vtx;
  if (c.pitMode || c.smartAudioVersion == VTX_SMARTAUDIO_V1) return;
  uint8_t frame[FRAME_SIZE] = {};
  send(frame, encodeClearPitMode(frame));
}

void VtxSmartAudio::sendPitModeFrequency()
{
  const auto& c = _model.config.vtx;
  const uint16_t f = vtxClampFrequency(c.pitModeFrequency);
  if (!f || c.smartAudioVersion == VTX_SMARTAUDIO_V1) return;
  uint8_t frame[FRAME_SIZE] = {};
  send(frame, encodePitModeFrequency(frame, f));
}

} // namespace Espfc::Connect
