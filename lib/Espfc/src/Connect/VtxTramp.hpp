#pragma once

#include "Connect/Vtx.hpp"
#include "Model.hpp"
#include "Stream/ReadWritable.hpp"
#include "Utils/Timer.hpp"

namespace Espfc::Connect {

// Fire and forget IRC Tramp sender, no response is read.
class VtxTramp
{
public:
  static constexpr size_t FRAME_SIZE = 16;
  static constexpr uint8_t BANDS = VTX_BAND_COUNT;
  static constexpr uint8_t CHANNELS = VTX_CHANNEL_COUNT;
  static constexpr uint8_t POWERS = 5;

  VtxTramp(Model& model): _model(model) {}

  int begin(Stream::ReadWritable* serial);
  int update();
  int update(uint32_t now);

  static void encode(uint8_t* frame, char cmd, uint16_t param);
  static uint16_t frequency(uint8_t band, uint8_t channel);
  static uint16_t power(uint8_t power);

private:
  enum class Command : uint8_t
  {
    FREQUENCY,
    POWER,
    PIT_MODE,
    COUNT,
  };

  void send(char cmd, uint16_t param);

  Stream::ReadWritable* _serial = nullptr;
  Model& _model;
  Command _next = Command::FREQUENCY;
  Utils::Timer _timer;
};

} // namespace Espfc::Connect
