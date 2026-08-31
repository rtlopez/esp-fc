#pragma once

#include "Stream/Readable.hpp"
#include "Stream/Writable.hpp"

namespace Espfc::Stream {

// Bidirectional byte stream, intended base for Device::SerialDevice.
class ReadWritable : public Readable, public Writable
{
protected:
  ~ReadWritable() = default;
};

} // namespace Espfc::Stream
