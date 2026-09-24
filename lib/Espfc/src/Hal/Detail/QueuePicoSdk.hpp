#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Hal::Detail {

// Type erased part of the pico-sdk backed queue. queue_t is stored by value in an opaque buffer,
// its real size and alignment are verified by static asserts in Hal/RP2040/QueuePicoSdk.cpp.
class QueuePicoSdkBase
{
public:
  static constexpr size_t STORAGE_SIZE = 24;
  static constexpr size_t STORAGE_ALIGN = 4;

protected:
  QueuePicoSdkBase(size_t elemSize, size_t count): _elemSize(elemSize), _count(count), _storage{} {}

  void beginImpl();
  bool pushBytes(const void* src);
  bool popBytes(void* dst);
  size_t sizeImpl() const;

  size_t _elemSize;
  size_t _count;
  alignas(STORAGE_ALIGN) uint8_t _storage[STORAGE_SIZE];
};

// Reference implementation, kept for comparison with QueueAtomic, see Hal/Queue.hpp.
template<typename T, size_t Capacity>
class QueuePicoSdk : private QueuePicoSdkBase
{
public:
  using ValueType = T;

  QueuePicoSdk(): QueuePicoSdkBase(sizeof(T), Capacity - 1) {}

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
