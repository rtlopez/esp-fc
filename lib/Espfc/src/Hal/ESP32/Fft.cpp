#if defined(ESP32)

#include "Hal/Dsp/Fft.hpp"
#include <cmath>
#include <cstdlib>
#include <dsps_fft2r.h>
#include <dsps_fft4r.h>
#include <dsps_wind_hann.h>
#include <esp_heap_caps.h>

namespace Espfc::Hal::Dsp {

namespace {

inline bool isPowerOfFour(size_t n)
{
  return n != 0 && (n & (n - 1)) == 0 && (n & 0x55555555UL) != 0;
}

} // namespace

void AlignedDeleter::operator()(float* ptr) const noexcept
{
  heap_caps_free(ptr);
}

FloatBuffer allocFloats(size_t count)
{
  void* ptr = heap_caps_aligned_alloc(16u, count * sizeof(float), MALLOC_CAP_DEFAULT);
  if (!ptr) std::abort();
  return FloatBuffer(static_cast<float*>(ptr));
}

bool Fft::init(size_t size)
{
  _size = size;
  const size_t bins = size >> 1;

  // cplx2real relies on the radix-4 twiddle table, so it must always be initialised.
  if (dsps_fft4r_init_fc32(nullptr, bins) != ESP_OK) return false;

  _radix4 = isPowerOfFour(bins);
  if (_radix4) return true;

  // bins is a power of two but not a power of four -> fall back to radix-2.
  return dsps_fft2r_init_fc32(nullptr, bins) == ESP_OK;
}

void Fft::windowHann(float* dst, size_t size)
{
  dsps_wind_hann_f32(dst, size);
}

void Fft::realForward(float* data)
{
  const size_t bins = _size >> 1;
  if (_radix4)
  {
    dsps_fft4r_fc32(data, bins);
    dsps_bit_rev4r_fc32(data, bins);
  }
  else
  {
    dsps_fft2r_fc32(data, bins);
    dsps_bit_rev_fc32(data, bins);
  }
  dsps_cplx2real_fc32(data, bins);
}

void Fft::magnitude(const float* src, float* dst)
{
  const size_t bins = _size >> 1;
  for (size_t i = 0; i < bins; i++)
  {
    const float re = src[2 * i];
    const float im = src[2 * i + 1];
    dst[i] = std::sqrt(re * re + im * im);
  }
}

} // namespace Espfc::Hal::Dsp

#endif
