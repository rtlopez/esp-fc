#include <platform.h>

#ifdef USE_FLASHFS

#include "Hal/FastCode.hpp"
#include "Hal/Flash.hpp"
#include "Utils/RingBuf.h"

static constexpr uint32_t FLASHFS_ERASED_VAL = 0xffffffff;

using BufferType = Espfc::Utils::RingBuf<uint8_t, FLASHFS_WRITE_BUFFER_SIZE>;

static BufferType buff;

static FlashfsRuntime flashfs;

const FlashfsRuntime* flashfsGetRuntime()
{
  return &flashfs;
}

static uint32_t FAST_CODE_ATTR flashfsJournalAddress(size_t index, bool getEnd = false)
{
  uint32_t journalBase = Espfc::Hal::Flash::getSize(flashfs.partition) - FLASHFS_JOURNAL_SIZE;
  return journalBase + (index * sizeof(FlashfsJournalItem)) + (getEnd * sizeof(uint32_t));
}

void flashfsJournalLoad(FlashfsJournalItem* data, size_t start, size_t num)
{
  size_t size = num * sizeof(FlashfsJournalItem);
  flashfsReadAbs(flashfsJournalAddress(start), (uint8_t*)data, size);
}

static void FAST_CODE_ATTR flashfsLogBegin()
{
  uint32_t beginAddr = flashfs.address;
  size_t idx = flashfs.journalIdx;

  flashfs.journal[idx].logBegin = beginAddr;

  if (!flashfs.partition) return;

  size_t address = flashfsJournalAddress(idx);

  flashfsWriteAbs(address, (uint8_t*)&beginAddr, sizeof(uint32_t));
}

static bool FAST_CODE_ATTR flashfsLogStarted()
{
  return flashfs.journal[flashfs.journalIdx].logBegin != FLASHFS_ERASED_VAL;
}

static void FAST_CODE_ATTR flashfsLogEnd()
{
  uint32_t endAddr = flashfs.address;
  size_t idx = flashfs.journalIdx;

  flashfs.journal[idx].logEnd = endAddr;

  flashfs.journalIdx++;

  if (!flashfs.partition) return;

  size_t address = flashfsJournalAddress(idx, true);

  flashfsWriteAbs(address, (uint8_t*)&endAddr, sizeof(uint32_t));
}

void flashfsJournalFix(FlashfsJournalItem* data, size_t num)
{
  uint8_t buff[4];
  for (size_t i = 0; i < num; i++)
  {
    if (data[i].logBegin != FLASHFS_ERASED_VAL && data[i].logEnd == FLASHFS_ERASED_VAL)
    {
      uint32_t addr = data[i].logBegin;
      uint32_t end = flashfsGetSize();
      while (addr < end)
      {
        flashfsReadAbs(addr, buff, 4);
        if (*reinterpret_cast<uint32_t*>(&buff[0]) == FLASHFS_ERASED_VAL)
        {
          flashfs.address = addr;
          flashfsLogEnd();
          break;
        }
        addr += 128;
      }
      break;
    }
  }
}

int flashfsInit(void)
{
  flashfs.partition = Espfc::Hal::Flash::findPartition();
  if (!flashfs.partition) return 0;

  flashfs.buffer = (void*)&buff;
  flashfs.journalIdx = 0;
  flashfs.address = 0;

  flashfsJournalLoad(flashfs.journal, 0, FLASHFS_JOURNAL_ITEMS);
  for (size_t i = 0; i < FLASHFS_JOURNAL_ITEMS; i++)
  {
    const auto& it = flashfs.journal[i];
    if (it.logEnd == FLASHFS_ERASED_VAL) break;
    flashfs.address = it.logEnd;
    flashfs.journalIdx++;
  }
  flashfsJournalFix(flashfs.journal, FLASHFS_JOURNAL_ITEMS);

  return 1;
}

bool FAST_CODE_ATTR flashfsFlushAsync(bool force)
{
  if (!flashfsLogStarted()) flashfsLogBegin();

  auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
  const size_t size = buffer->size();
  if (flashfs.partition && size > 0)
  {
    // uint32_t newAddress = force ? (flashfs.address + size) : Espfc::Utils::alignAddressToWrite(flashfs.address, size,
    // FLASHFS_FLUSH_BUFFER_SIZE); size_t toWrite = newAddress - flashfs.address;
    uint8_t tmp[FLASHFS_FLUSH_BUFFER_SIZE];
    size_t chunks = (size / FLASHFS_FLUSH_BUFFER_SIZE) + 1;
    while (chunks--)
    {
      size_t len = buffer->pop(tmp, FLASHFS_FLUSH_BUFFER_SIZE);
      flashfsWriteAbs(flashfs.address, tmp, len);
      flashfs.address += len;
    }
    return true;
  }
  return false;
}

void FAST_CODE_ATTR flashfsWriteByte(uint8_t byte)
{
  if (flashfsIsEOF()) return;

  auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
  buffer->push(byte);
  const size_t size = buffer->size();

  if (flashfs.address + size >= flashfsGetSize())
  {
    flashfsClose();
    return;
  }

  if ((flashfs.address + size) % Espfc::Hal::Flash::getSectorSize() == 0)
  {
    flashfsFlushAsync(true);
    return;
  }

  if (size >= FLASHFS_FLUSH_BUFFER_SIZE)
  {
    flashfsFlushAsync(false);
  }
}

void FAST_CODE_ATTR flashfsWrite(const uint8_t* data, unsigned int len, bool sync)
{
  (void)sync;
  while (len)
  {
    flashfsWriteByte(*data);
    data++;
    len--;
  }
}

void FAST_CODE_ATTR flashfsWriteAbs(uint32_t address, const uint8_t* data, unsigned int len)
{
  Espfc::Hal::Flash::write(flashfs.partition, address, data, len);
}

int FAST_CODE_ATTR flashfsReadAbs(uint32_t address, uint8_t* data, unsigned int len)
{
  return Espfc::Hal::Flash::read(flashfs.partition, address, data, len);
}

bool FAST_CODE_ATTR flashfsIsSupported(void)
{
  return (bool)flashfs.partition;
}

bool FAST_CODE_ATTR flashfsIsReady(void)
{
  return flashfsIsSupported();
}

bool FAST_CODE_ATTR flashfsIsEOF(void)
{
  return !flashfs.partition || flashfs.journalIdx >= FLASHFS_JOURNAL_ITEMS || flashfs.address >= flashfsGetSize();
}

void flashfsEraseCompletely(void)
{
  Espfc::Hal::Flash::erase(flashfs.partition, 0, Espfc::Hal::Flash::getSize(flashfs.partition));
  flashfsInit();
}

void FAST_CODE_ATTR flashfsClose(void)
{
  flashfsFlushAsync(true);
  flashfsLogEnd();
}

uint32_t FAST_CODE_ATTR flashfsGetWriteBufferFreeSpace(void)
{
  auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
  return buffer->available();
}

uint32_t FAST_CODE_ATTR flashfsGetWriteBufferSize(void)
{
  return FLASHFS_WRITE_BUFFER_SIZE;
}

uint32_t FAST_CODE_ATTR flashfsGetSize(void)
{
  const uint32_t size = Espfc::Hal::Flash::getSize(flashfs.partition);
  return size ? size - FLASHFS_JOURNAL_SIZE : 0;
}

uint32_t flashfsGetOffset(void)
{
  return flashfs.address;
}

uint32_t flashfsGetSectors(void)
{
  return flashfsGetSize() / Espfc::Hal::Flash::getSectorSize();
}

#endif
