#pragma once

#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "Model.h"
#include "Stream/ReadWritable.hpp"
#include <functional>

namespace Espfc::Connect {

class MspProcessor
{
public:
  MspProcessor(Model& model);
  bool parse(char c, MspMessage& msg);
  void processCommand(MspMessage& m, MspResponse& r, Stream::ReadWritable& s);
  void processEsc4way();
  void processRestart();
  void serializeFlashData(MspResponse& r, uint32_t address, const uint16_t size, bool useLegacyFormat,
                          bool allowCompression);

  void sendResponse(MspResponse& r, Stream::ReadWritable& s);
  void postCommand();
  bool debugSkip(uint8_t cmd);
  void debugMessage(const MspMessage& m);
  void debugResponse(const MspResponse& r);

private:
  Model& _model;
  MspParser _parser;
  std::function<void(void)> _postCommand;
};

} // namespace Espfc::Connect
