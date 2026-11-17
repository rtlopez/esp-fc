#include "VtxTramp.hpp"
#include "Hal/Time.hpp"
#include <algorithm>

namespace Espfc::Connect {

namespace {

constexpr uint32_t REFRESH_US = 400000;

constexpr uint16_t FREQUENCIES[VtxTramp::BANDS][VtxTramp::CHANNELS] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // F
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // R
};

constexpr uint16_t POWERS_MW[VtxTramp::POWERS] = {25, 100, 200, 400, 600};

} // namespace

int VtxTramp::begin(Stream::ReadWritable* serial)
{
  _serial = serial;
  _next = Command::FREQUENCY;
  _timer.setInterval(REFRESH_US / static_cast<uint32_t>(Command::COUNT));
  _model.state.vtx.type = VTXDEV_TRAMP;
  _model.state.vtx.active = serial != nullptr;
  return 1;
}

int VtxTramp::update()
{
  return update(micros());
}

int VtxTramp::update(uint32_t now)
{
  if (!_serial || !_timer.check(now)) return 1;

  const auto& c = _model.config.vtx;
  switch (_next)
  {
    case Command::FREQUENCY: {
      uint16_t f = vtxDirectFrequency(c);
      if (!f && c.band) f = frequency(c.band, c.channel);
      if (f) send('F', f);
      _next = Command::POWER;
      break;
    }
    case Command::POWER:
      send('P', power(c.lowPowerDisarm && !_model.isModeActive(MODE_ARMED) ? 1 : c.power));
      _next = Command::PIT_MODE;
      break;
    default:
      send('I', c.pitMode ? 0 : 1);
      _next = Command::FREQUENCY;
      break;
  }
  return 1;
}

void VtxTramp::encode(uint8_t* frame, char cmd, uint16_t param)
{
  std::fill(frame, frame + FRAME_SIZE, 0);
  frame[0] = 0x0F;
  frame[1] = static_cast<uint8_t>(cmd);
  frame[2] = param & 0xFF;
  frame[3] = param >> 8;
  uint8_t sum = 0;
  for (size_t i = 1; i < 14; i++)
  {
    sum += frame[i];
  }
  frame[14] = sum;
}

uint16_t VtxTramp::frequency(uint8_t band, uint8_t channel)
{
  return FREQUENCIES[std::clamp<uint8_t>(band, 1, BANDS) - 1][std::clamp<uint8_t>(channel, 1, CHANNELS) - 1];
}

uint16_t VtxTramp::power(uint8_t power)
{
  return POWERS_MW[std::clamp<uint8_t>(power, 1, POWERS) - 1];
}

void VtxTramp::send(char cmd, uint16_t param)
{
  if (_serial->availableForWrite() < static_cast<int>(FRAME_SIZE)) return;
  uint8_t frame[FRAME_SIZE];
  encode(frame, cmd, param);
  _serial->write(frame, FRAME_SIZE);
}

} // namespace Espfc::Connect
