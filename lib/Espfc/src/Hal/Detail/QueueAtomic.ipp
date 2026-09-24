#pragma once

namespace Espfc::Hal::Detail {

template<typename T, size_t Capacity, typename Index>
inline bool QueueAtomic<T, Capacity, Index>::push(const T& item)
{
  const size_t tail = _tail.loadRelaxed();
  const size_t nextTail = next(tail);
  if (nextTail == _head.load()) return false;

  _buffer[tail] = item;
  _tail.store(nextTail); // release, publishes the payload written above
  return true;
}

template<typename T, size_t Capacity, typename Index>
inline bool QueueAtomic<T, Capacity, Index>::pop(T& item)
{
  const size_t head = _head.loadRelaxed();
  if (head == _tail.load()) return false;

  item = _buffer[head];
  _head.store(next(head)); // release, frees the slot read above
  return true;
}

template<typename T, size_t Capacity, typename Index>
inline bool QueueAtomic<T, Capacity, Index>::isEmpty() const
{
  return _head.load() == _tail.load();
}

template<typename T, size_t Capacity, typename Index>
inline bool QueueAtomic<T, Capacity, Index>::isFull() const
{
  return next(_tail.load()) == _head.load();
}

template<typename T, size_t Capacity, typename Index>
inline size_t QueueAtomic<T, Capacity, Index>::size() const
{
  return (_tail.load() - _head.load()) & (Capacity - 1);
}

} // namespace Espfc::Hal::Detail
