#pragma once

#include <atomic>
#include <type_traits>

namespace Espfc::Utils {

template<typename T>
class SeqLockWrapper
{
  static_assert(std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T> && std::is_default_constructible_v<T>,
                "T Must be copyable");

public:
  SeqLockWrapper(): _counter(0), _data{} {}
  explicit SeqLockWrapper(const T& data): _counter(0), _data(data) {}

  SeqLockWrapper(const SeqLockWrapper& other)
  {
    this->_counter.store(other._counter.load(std::memory_order_relaxed), std::memory_order_relaxed);
    this->_data = other._data;
  }

  SeqLockWrapper& operator=(const SeqLockWrapper& other)
  {
    if (this != &other)
    {
      this->_counter.store(other._counter.load(std::memory_order_relaxed), std::memory_order_relaxed);
      this->_data = other._data;
    }
    return *this;
  }

  SeqLockWrapper(SeqLockWrapper&& other) noexcept
  {
    this->_counter.store(other._counter.load(std::memory_order_relaxed), std::memory_order_relaxed);
    this->_data = std::move(other._data);
  }

  SeqLockWrapper& operator=(SeqLockWrapper&& other) noexcept
  {
    if (this != &other)
    {
      this->_counter.store(other._counter.load(std::memory_order_relaxed), std::memory_order_relaxed);
      this->_data = std::move(other._data);
    }
    return *this;
  }

  // STORE (Producers)
  void store(const T& data)
  {
    uint32_t curr = _counter.load(std::memory_order_relaxed);

    // 1. open section (odd counter)
    _counter.store(curr + 1, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_seq_cst); // Bariera sprzętowa dla RP2xxx

    // 2. store new _data
    _data = data;

    // 3. close section (even counter)
    std::atomic_thread_fence(std::memory_order_seq_cst);
    _counter.store(curr + 2, std::memory_order_release);
  }

  // FETCH (Consumers)
  T fetch() const
  {
    T data;
    uint32_t seqBefore = 0;
    uint32_t seqAfter = 0;

    do
    {
      seqBefore = _counter.load(std::memory_order_acquire);

      if (seqBefore & 1)
      {
// if seqBefore is odd, it means a write is in progress, so we should wait and try again
#if defined(__arm__) || defined(__thumb__)
        __asm volatile("yield"); // or "nop" or "wfe" depending on the architecture
#else
        __asm volatile("nop");
#endif
        continue;
      }

      std::atomic_thread_fence(std::memory_order_seq_cst);

      // Copy _data to local variable
      data = _data;

      std::atomic_thread_fence(std::memory_order_seq_cst);
      seqAfter = _counter.load(std::memory_order_acquire);

    } while (seqBefore != seqAfter);

    return data;
  }

private:
  alignas(4) std::atomic<uint32_t> _counter;
  alignas(alignof(T)) T _data;
};

} // namespace Espfc::Utils
