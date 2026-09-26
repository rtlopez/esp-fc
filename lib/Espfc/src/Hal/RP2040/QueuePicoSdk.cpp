#if defined(ARCH_RP2040)

#include "Hal/Detail/QueuePicoSdk.hpp"
#include "Hal/FastCode.hpp"
#include <pico/util/queue.h>

namespace Espfc::Hal::Detail {

static_assert(sizeof(queue_t) <= QueuePicoSdkBase::STORAGE_SIZE, "opaque storage too small");
static_assert(alignof(queue_t) <= QueuePicoSdkBase::STORAGE_ALIGN, "opaque storage misaligned");

static inline queue_t* handle(uint8_t* storage)
{
  return reinterpret_cast<queue_t*>(storage);
}

void QueuePicoSdkBase::beginImpl()
{
  queue_init(handle(_storage), _elemSize, _count);
}

bool FAST_CODE_ATTR QueuePicoSdkBase::pushBytes(const void* src)
{
  return queue_try_add(handle(_storage), src);
}

bool FAST_CODE_ATTR QueuePicoSdkBase::popBytes(void* dst)
{
  return queue_try_remove(handle(_storage), dst);
}

size_t FAST_CODE_ATTR QueuePicoSdkBase::sizeImpl() const
{
  return queue_get_level(handle(const_cast<uint8_t*>(_storage)));
}

} // namespace Espfc::Hal::Detail

#endif
