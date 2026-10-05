#if defined(ESP32)

#include "Hal/Dsp/Fft.hpp"
#include <cstdlib>
#include <dsps_fft4r.h>
#include <dsps_wind_hann.h>
#include <esp_heap_caps.h>

namespace Espfc::Hal::Dsp {

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
  return dsps_fft4r_init_fc32(nullptr, size >> 1) == ESP_OK;
}

void Fft::windowHann(float* dst, size_t size)
{
  dsps_wind_hann_f32(dst, size);
}

void Fft::realForward(float* data)
{
  // FFT Radix-4
  const size_t bins = _size >> 1;
  dsps_fft4r_fc32(data, bins);
  dsps_bit_rev4r_fc32(data, bins);
  dsps_cplx2real_fc32(data, bins);
}

} // namespace Espfc::Hal::Dsp

#endif
