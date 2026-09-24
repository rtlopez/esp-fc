#pragma once

#include "Hal/Detail/QueueIndex.hpp"
#include <cstddef>
#include <type_traits>

namespace Espfc::Hal::Detail {

// Single producer, single consumer ring buffer. Synchronization is provided by the Index policy,
// the algorithm itself uses plain load and store only, no read-modify-write operations.
template<typename T, size_t Capacity, typename Index = DefaultIndex>
class QueueAtomic
{
  static_assert(Capacity >= 2, "capacity must be at least 2");
  static_assert((Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");
  static_assert(std::is_trivially_copyable_v<T>, "element must be trivially copyable");

public:
  using ValueType = T;

  static constexpr size_t capacity()
  {
    return Capacity - 1; // one slot is used to tell an empty queue from a full one
  }

  void begin() {}

  bool push(const T& item);
  bool pop(T& item);
  bool isEmpty() const;
  bool isFull() const;
  size_t size() const;

private:
  static constexpr size_t next(size_t index)
  {
    return (index + 1) & (Capacity - 1);
  }

  Index _head; // owned by the consumer
  Index _tail; // owned by the producer
  T _buffer[Capacity];
};

} // namespace Espfc::Hal::Detail

#include "Hal/Detail/QueueAtomic.ipp"
