#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Stream {

// Minimal byte sink interface, framework independent counterpart of Arduino Print.
// Formatting is intentionally not part of it, see Stream::Printer.
class Writable
{
public:
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* data, size_t len) = 0;
  virtual int availableForWrite() = 0;
  virtual void flush() = 0;
  virtual bool isTxFifoEmpty() = 0;

protected:
  // Non virtual and protected on purpose: no vtable slot, no delete through base pointer.
  ~Writable() = default;
};

} // namespace Espfc::Stream
