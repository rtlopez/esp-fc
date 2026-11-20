#pragma once

#include "Connect/Vtx.hpp"
#include "Model.hpp"
#include "Stream/ReadWritable.hpp"
#include "Utils/Timer.hpp"

namespace Espfc::Connect {

// Fire and forget TBS SmartAudio sender, no response is read.
class VtxSmartAudio
{
public:
  static constexpr size_t FRAME_SIZE = 7;
  static constexpr uint8_t BANDS = VTX_BAND_COUNT;
  static constexpr uint8_t CHANNELS = VTX_CHANNEL_COUNT;
  static constexpr uint8_t POWERS = 4;

  VtxSmartAudio(Model& model): _model(model) {}

  int begin(Stream::ReadWritable* serial);
  int update();
  int update(uint32_t now);

  static size_t encodeChannel(uint8_t* frame, uint8_t band, uint8_t channel);
  static size_t encodePower(uint8_t* frame, uint8_t power, int8_t version);
  static size_t encodePitModePower(uint8_t* frame);
  static size_t encodeClearPitMode(uint8_t* frame);
  static size_t encodeFrequency(uint8_t* frame, uint16_t frequency);
  static size_t encodePitModeFrequency(uint8_t* frame, uint16_t frequency);

private:
  enum class Command : uint8_t
  {
    CHANNEL,
    POWER,
    MODE,
    PIT_FREQUENCY,
    COUNT,
  };

  void send(const uint8_t* frame, size_t len);
  void sendChannel();
  void sendPower();
  void sendMode();
  void sendPitModeFrequency();

  Stream::ReadWritable* _serial = nullptr;
  Model& _model;
  Command _next = Command::CHANNEL;
  Utils::Timer _timer;
};

} // namespace Espfc::Connect
