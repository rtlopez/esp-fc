#pragma once

#include "BlackboxSerialBuffer.h"
#include "Model.h"
#include "Stream/ReadWritable.hpp"

namespace Espfc::Blackbox {

class Blackbox
{
public:
  Blackbox(Model& model);
  int begin();
  int update();

private:
  void updateData();
  void updateArmed();
  void updateMode();

  Model& _model;
  Stream::ReadWritable* _serial;
  BlackboxSerialBuffer _buffer;
  uint32_t _lastGpsTs = 0;
};

} // namespace Espfc::Blackbox
