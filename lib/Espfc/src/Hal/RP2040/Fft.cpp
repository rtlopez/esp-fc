#if defined(ARCH_RP2350)

#include "Hal/Dsp/Fft.hpp"
#include "arm_math.h"
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace Espfc::Hal::Dsp {

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
  _size = size;
  if (!_scratch) _scratch = allocFloats(size);
  arm_rfft_fast_instance_f32 inst;
  return arm_rfft_fast_init_f32(&inst, static_cast<uint16_t>(size)) == ARM_MATH_SUCCESS;
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
  arm_rfft_fast_instance_f32 inst;
  arm_rfft_fast_init_f32(&inst, static_cast<uint16_t>(_size));
  arm_rfft_fast_f32(&inst, data, _scratch.get(), 0);
  std::memcpy(data, _scratch.get(), _size * sizeof(float));
}

} // namespace Espfc::Hal::Dsp

#endif
