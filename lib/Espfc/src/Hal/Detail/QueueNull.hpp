#pragma once

#include <cstddef>

namespace Espfc::Hal::Detail {

// Queue that drops everything, for single core targets where events are dispatched inline.
template<typename T, size_t Capacity>
class QueueNull
{
public:
  using ValueType = T;

  static constexpr size_t capacity()
  {
    return 0;
  }

  void begin() {}

  bool push(const T&)
  {
    return false;
  }

  bool pop(T&)
  {
    return false;
  }

  bool isEmpty() const
  {
    return true;
  }

  bool isFull() const
  {
    return false;
  }

  size_t size() const
  {
    return 0;
  }
};

} // namespace Espfc::Hal::Detail
