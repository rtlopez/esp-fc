#if defined(ARCH_RP2350)

#include "Hal/Dsp/Fft.hpp"
#include <arm_math.h>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace Espfc::Hal::Dsp {

namespace {

// RFFT config is immutable per length (only const twiddle/bit-rev pointers), so every axis
// sharing a length reuses one instance. Mirrors the ESP32 backend's global twiddle tables.
struct RfftEntry
{
  uint16_t len;
  bool ready;
  arm_rfft_fast_instance_f32 inst;
};

RfftEntry g_rfft[6] = {};

const arm_rfft_fast_instance_f32* rfftFind(uint16_t len)
{
  for (auto& e : g_rfft)
  {
    if (e.ready && e.len == len) return &e.inst;
  }
  return nullptr;
}

const arm_rfft_fast_instance_f32* rfftEnsure(uint16_t len)
{
  if (const arm_rfft_fast_instance_f32* inst = rfftFind(len)) return inst;
  for (auto& e : g_rfft)
  {
    if (!e.ready)
    {
      if (arm_rfft_fast_init_f32(&e.inst, len) != ARM_MATH_SUCCESS) return nullptr;
      e.len = len;
      e.ready = true;
      return &e.inst;
    }
  }
  return nullptr;
}

} // namespace

void AlignedDeleter::operator()(float* ptr) const noexcept
{
  std::free(ptr);
}

FloatBuffer allocFloats(size_t count)
{
  void* ptr = std::malloc(count * sizeof(float));
  if (!ptr) std::abort();
  return FloatBuffer(static_cast<float*>(ptr));
}

bool Fft::init(size_t size)
{
  // reallocate only if size has changed
  if (_size != size)
  {
    _size = size;
    _scratch = allocFloats(size);
  }
  return rfftEnsure(static_cast<uint16_t>(size)) != nullptr;
}

void Fft::windowHann(float* dst, size_t size)
{
  const float twoPi = 6.28318530717959f;
  const float denom = (size > 1) ? static_cast<float>(size - 1) : 1.0f;
  for (size_t i = 0; i < size; i++)
  {
    dst[i] = 0.5f * (1.0f - cosf(twoPi * static_cast<float>(i) / denom));
  }
}

void Fft::realForward(float* data)
{
  // arm_rfft_fast_f32 is out-of-place; transform into scratch then restore in-place contract.
  const auto* inst = rfftFind(static_cast<uint16_t>(_size));
  arm_rfft_fast_f32(inst, data, _scratch.get(), 0);
  std::memcpy(data, _scratch.get(), _size * sizeof(float));
}

} // namespace Espfc::Hal::Dsp

#endif
