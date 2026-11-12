#pragma once

#include "Stream/ReadWritable.hpp"
#include "Utils/Timer.hpp"

namespace Espfc::Output {

class OutputIBUS
{
public:
  OutputIBUS();

  int begin(Stream::ReadWritable* serial);
  int update();

private:
  Stream::ReadWritable* _serial;
  Utils::Timer _timer;
};

} // namespace Espfc::Output
