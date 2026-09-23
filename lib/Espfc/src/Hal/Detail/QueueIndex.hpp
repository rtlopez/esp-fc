#pragma once

#include "Hal/Platform.hpp"
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace Espfc::Hal::Detail {

// Index without any synchronization, for single core targets.
class PlainIndex
{
public:
  uint32_t load() const
  {
    return _v;
  }

  uint32_t loadRelaxed() const
  {
    return _v;
  }

  void store(uint32_t v)
  {
    _v = v;
  }

private:
  uint32_t _v{0};
};

#if defined(__ARM_ARCH_6M__)

// ARMv6-M (Cortex-M0+, RP2040) has no LDREX/STREX, so read-modify-write atomics fall back to library calls
// guarded by a spinlock. Aligned 32 bit load and store are atomic in hardware, the only missing piece is
// ordering, which DMB provides. Both cores see the same uncached SRAM, so no cache maintenance is needed.
class BarrierIndex
{
public:
  uint32_t load() const
  {
    const uint32_t v = *static_cast<const volatile uint32_t*>(&_v);
    __asm volatile("dmb" ::: "memory");
    return v;
  }

  uint32_t loadRelaxed() const
  {
    return *static_cast<const volatile uint32_t*>(&_v);
  }

  void store(uint32_t v)
  {
    __asm volatile("dmb" ::: "memory");
    *static_cast<volatile uint32_t*>(&_v) = v;
  }

private:
  alignas(4) uint32_t _v{0};
};

using DefaultIndex = BarrierIndex;

#elif ATOMIC_INT_LOCK_FREE == 2

// Index based on lock free atomics, for targets with full atomic support.
class AtomicIndex
{
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "lock free atomics required");

public:
  uint32_t load() const
  {
    return _v.load(std::memory_order_acquire);
  }

  uint32_t loadRelaxed() const
  {
    return _v.load(std::memory_order_relaxed);
  }

  void store(uint32_t v)
  {
    _v.store(v, std::memory_order_release);
  }

private:
  std::atomic<uint32_t> _v{0};
};

using DefaultIndex = std::conditional_t<MULTI_CORE, AtomicIndex, PlainIndex>;

#else

// Targets without lock free atomics, eg. esp8266, esp32s2, esp32c3. All of them are single core,
// so no synchronization is needed at all.
static_assert(!MULTI_CORE, "multi core target without lock free atomics needs a dedicated index");

using DefaultIndex = PlainIndex;

#endif

} // namespace Espfc::Hal::Detail
