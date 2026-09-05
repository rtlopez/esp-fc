#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Stream {

// Minimal byte source interface, framework independent counterpart of Arduino Stream reads.
class Readable
{
public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t readMany(uint8_t* data, size_t len) = 0;
  virtual int peek() = 0;

protected:
  // Non virtual and protected on purpose: no vtable slot, no delete through base pointer.
  ~Readable() = default;
};

} // namespace Espfc::Stream
