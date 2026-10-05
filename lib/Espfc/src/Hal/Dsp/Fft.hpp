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
class Fft
{
public:
  bool init(size_t size);
  void windowHann(float* dst, size_t size);
  void realForward(float* data);

private:
  size_t _size = 0;
};

} // namespace Espfc::Hal::Dsp
