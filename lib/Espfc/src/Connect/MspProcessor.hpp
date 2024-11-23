#pragma once

#include "Model.h"
#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "Device/SerialDevice.h"
#include <functional>

namespace Espfc {

namespace Connect {

class MspProcessor
{
public:
  MspProcessor(Model& model);
  bool parse(char c, MspMessage& msg);
  void processCommand(MspMessage& m, MspResponse& r, Device::SerialDevice& s);
  void processEsc4way();
  void processRestart();
  void serializeFlashData(MspResponse& r, uint32_t address, const uint16_t size, bool useLegacyFormat, bool allowCompression);

  void sendResponse(MspResponse& r, Device::SerialDevice& s);
  void postCommand();
  bool debugSkip(uint8_t cmd);
  void debugMessage(const MspMessage& m);
  void debugResponse(const MspResponse& r);

private:
  Model& _model;
  MspParser _parser;
  std::function<void(void)> _postCommand;
};

}

}
