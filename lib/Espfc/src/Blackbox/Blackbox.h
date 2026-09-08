#pragma once

#include "BlackboxSerialBuffer.h"
#include "Model.h"
#include "Stream/ReadWritable.hpp"
extern "C" {
#include <platform.h>
}

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
  pidProfile_s _pidProfile;
  Stream::ReadWritable* _serial;
  BlackboxSerialBuffer _buffer;
};

} // namespace Espfc::Blackbox
