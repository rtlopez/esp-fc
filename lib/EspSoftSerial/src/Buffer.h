#ifndef _BUFFER_H_
#define _BUFFER_H_

#include <stddef.h>

template <class T, size_t S>
class Buffer {
public:
  Buffer(): _head(0), _tail(0), _count(0) {}

	void push(const T& item)
	{
		_buf[_head] = item;
		_head = (_head + 1) % S;
		if(_head == _tail)
		{
			_tail = (_tail + 1) % S;
		}
    _count++;
    if(_count > S - 1) _count = S - 1;
	}

	T get()
	{
    if(empty()) return T();

		//Read data and advance the tail (we now have a free space)
		T val = _buf[_tail];
		_tail = (_tail + 1) % S;

    _count--;
    if(_count < 0) _count = 0;

		return val;
	}

  T peek(void) const
	{
		if(empty()) return T();
    return _buf[_tail];
  }

	void reset(void)
	{
		_head = _tail;
	}

	bool empty(void) const
	{
		//if head and tail are equal, we are empty
		return _head == _tail;
	}

	bool full(void) const
	{
		//If tail is ahead the head by 1, we are full
		return ((_head + 1) % S) == _tail;
	}

  int count() const
  {
    return _count;
  }

  size_t size() const
	{
		return S - 1;
	}

private:
	size_t _head;
	size_t _tail;
  int _count;
  T _buf[S];
};

#endif
