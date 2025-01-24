#pragma once

#include "Utils/Sma.hpp"

namespace Espfc {

namespace Utils {

template<typename SampleType, size_t MaxSize>
Sma<SampleType, MaxSize>::Sma(): _idx(0), _count(MaxSize)
{
  begin(MaxSize);
}

template<typename SampleType, size_t MaxSize>
void Sma<SampleType, MaxSize>::begin(size_t count)
{
  _count = std::min(count, MaxSize);
  _inv_count = 1.f / _count;
}

template<typename SampleType, size_t MaxSize>
SampleType Sma<SampleType, MaxSize>::update(const SampleType& input)
{
  if(_count > 1)
  {
    _sum -= _samples[_idx];
    _sum += input;
    _samples[_idx] = input;
    if (++_idx >= _count) _idx = 0;
    return _sum * _inv_count;
  }
  return input;
}

}

}
