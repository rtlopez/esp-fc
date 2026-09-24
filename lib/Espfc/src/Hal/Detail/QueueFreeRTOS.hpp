#pragma once

#include <cstddef>

namespace Espfc::Hal::Detail {

// Type erased part of the FreeRTOS backed queue, kept out of line so that no FreeRTOS header
// is needed here. QueueHandle_t is a plain pointer, so void* is enough to store it.
class QueueFreeRtosBase
{
protected:
  QueueFreeRtosBase(size_t elemSize, size_t count): _elemSize(elemSize), _count(count), _handle(nullptr) {}

  void beginImpl();
  bool pushBytes(const void* src);
  bool popBytes(void* dst);
  size_t sizeImpl() const;

  size_t _elemSize;
  size_t _count;
  void* _handle;
};

// Reference implementation, kept for comparison with QueueAtomic, see Hal/Queue.hpp.
template<typename T, size_t Capacity>
class QueueFreeRTOS : private QueueFreeRtosBase
{
public:
  using ValueType = T;

  QueueFreeRTOS(): QueueFreeRtosBase(sizeof(T), Capacity - 1) {}

  static constexpr size_t capacity()
  {
    return Capacity - 1;
  }

  void begin()
  {
    beginImpl();
  }

  bool push(const T& item)
  {
    return pushBytes(&item);
  }

  bool pop(T& item)
  {
    return popBytes(&item);
  }

  bool isEmpty() const
  {
    return sizeImpl() == 0;
  }

  bool isFull() const
  {
    return sizeImpl() >= capacity();
  }

  size_t size() const
  {
    return sizeImpl();
  }
};

} // namespace Espfc::Hal::Detail
