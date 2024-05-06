#pragma once

#include <cstddef>

// https://gist.github.com/edwintcloud/d547a4f9ccaf7245b06f0e8782acefaa

namespace Espfc {

namespace Utils {

template<typename Element, size_t Size>
class RingBuf
{
public:
  enum { Capacity = Size + 1 };

  RingBuf(): _tail(0), _head(0) {}
  ~RingBuf() {}

  bool push(const Element& item)
  {
    if(isFull()) return false;

    _array[_tail] = item;
    _tail = _increment(_tail);

    return true;
  }

  size_t push(const Element *items, size_t len)
  {
    len = std::min(available(), len);
    for(size_t i = 0; i < len; i++)
    {
      _array[_tail] = items[i];
      _tail = _increment(_tail);
    }
    return len;
  }

  bool pop(Element& item)
  {
    if(isEmpty()) return false;

    item = _array[_head];
    _head = _increment(_head);

    return true;
  }

  size_t pop(Element * items, size_t len)
  {
    len = std::min(size(), len);
    for(size_t i = 0; i < len; i++)
    {
      items[i] = _array[_head];
      _head = _increment(_head);
    }
    return len;
  }

  bool isEmpty() const
  {
    return _head == _tail;
  }

  bool isFull() const
  {
    return _head == _increment(_tail);
  }

  size_t size() const
  {
    if (_tail >= _head) return _tail - _head;
    return Capacity - (_head - _tail);
  }

  size_t available() const
  {
    return Size - size();
  }

private:
  size_t _increment(size_t idx) const { return (idx + 1) % Capacity; } 
  size_t _tail;
  size_t _head; 
  Element _array[Capacity];
};

}

}
