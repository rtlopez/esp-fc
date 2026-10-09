#pragma once

#include <cstddef>
#include <memory>

namespace Espfc::Hal::Dsp {

// Frees a buffer obtained from allocFloats() with the platform aligned allocator.
struct AlignedDeleter
{
  void operator()(float* ptr) const noexcept;
};

using FloatBuffer = std::unique_ptr<float[], AlignedDeleter>;

// Aligned float buffer suitable for the platform FFT backend.
FloatBuffer allocFloats(size_t count);

// Forward real FFT with float samples.
// Contract: size is the number of real input samples (power of two).
// realForward() is in-place: size reals become size floats holding size/2
// interleaved complex bins (bin k at [2*k] = Re, [2*k + 1] = Im).
// CMSIS-DSP backend pack the nyquist real value in data[1], take into account when calculating DC magnitude.
// magnitude() converts realForward() output into size/2 bin magnitudes (dst[0] = |DC|), dst may alias src.
class Fft
{
public:
  bool init(size_t size);
  void windowHann(float* dst, size_t size);
  void realForward(float* data);
  void magnitude(const float* src, float* dst);

private:
  size_t _size = 0;
  bool _radix4 = true;  // radix-4 backend selected, false selects radix-2 fallback
  FloatBuffer _scratch; // out-of-place backends (e.g. CMSIS-DSP) only
};

} // namespace Espfc::Hal::Dsp
