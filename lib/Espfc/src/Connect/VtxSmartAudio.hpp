#pragma once

#include "Connect/Vtx.hpp"
#include "Model.hpp"
#include "Utils/Timer.hpp"

namespace Espfc::Connect {

// Fire and forget TBS SmartAudio sender, no response is read.
class VtxSmartAudio
{
public:
  static constexpr size_t FRAME_SIZE = 6;
  static constexpr uint8_t BANDS = 5;
  static constexpr uint8_t CHANNELS = 8;

  VtxSmartAudio(Model& model): _model(model) {}

  int begin(Stream::ReadWritable* serial);
  int update();
  int update(uint32_t now);

  static void encodeChannel(uint8_t* frame, uint8_t band, uint8_t channel);
  static void encodePower(uint8_t* frame, uint8_t power);

private:
  enum class Command : uint8_t
  {
    CHANNEL,
    POWER,
    COUNT,
  };

  void send(const uint8_t* frame);
  void sendChannel();
  void sendPower();

  Stream::ReadWritable* _serial = nullptr;
  Model& _model;
  Command _next = Command::CHANNEL;
  Utils::Timer _timer;
};

} // namespace Espfc::Connect
