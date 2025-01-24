#pragma once

#include <cstddef>

namespace Espfc {

namespace Utils {

template<typename SampleType, size_t MaxSize>
class Sma
{
public:
  Sma();
  void begin(size_t count);
  SampleType update(const SampleType& input);

private:
  SampleType _samples[MaxSize];
  SampleType _sum;
  size_t _idx;
  size_t _count;
  float _inv_count;
};

}

}
