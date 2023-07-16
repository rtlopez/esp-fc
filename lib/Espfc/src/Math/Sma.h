#ifndef _ESPFC_MATH_SMA_H_
#define _ESPFC_MATH_SMA_H_

namespace Espfc {

namespace Math {

template<typename SampleType, size_t MaxSize>
class Sma
{
public:
  Sma(): _idx(0), _count(MaxSize)
  {
    begin(MaxSize);
  }

  void begin(size_t count)
  {
    _count = std::min(count, MaxSize);
    _inv_count = 1.f / _count;
  }

  SampleType update(const SampleType& input)
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

private:
  SampleType _samples[MaxSize];
  SampleType _sum;
  size_t _idx;
  size_t _count;
  float _inv_count;
};

}

}

#endif