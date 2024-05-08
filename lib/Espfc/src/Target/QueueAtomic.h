#pragma once

#if defined(ESPFC_ATOMIC_QUEUE) || defined(UNIT_TEST)

#include <cstddef>
#include <atomic>

namespace Espfc {

template<typename Element, size_t Size>
class QueueAtomic
{
public:
  enum { Capacity = Size + 1 };

  QueueAtomic(): _tail(0), _head(0) {}
  ~QueueAtomic() {}

  bool push(const Element& item)
  {	
    auto current_tail = _tail.load();            
    auto next_tail = increment(current_tail);    
    if(next_tail != _head.load())                         
    {
      _array[current_tail] = item;               
      _tail.store(next_tail);                    
      return true;
    }
    return false;  // full queue
  }

  bool pop(Element& item)
  {
    const auto current_head = _head.load();
    if(current_head == _tail.load()) return false;   // empty queue

    item = _array[current_head]; 
    _head.store(increment(current_head)); 
    return true;
  }

  bool isEmpty() const { return (_head.load() == _tail.load()); }

  bool isFull() const
  {
    const auto next_tail = increment(_tail.load());
    return (next_tail == _head.load());
  }
  bool isLockFree() const { return std::atomic<Element>{}.is_lock_free(); }

private:
  size_t increment(size_t idx) const { return (idx + 1) % Capacity; } 

  std::atomic<size_t> _tail;
  std::atomic<size_t> _head;
  Element _array[Capacity];
};

}

#endif
