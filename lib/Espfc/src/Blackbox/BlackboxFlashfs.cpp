#include <Arduino.h>
#include <platform.h>

#ifdef USE_FLASHFS

#include "Device/FlashDevice.h"
#include "Utils/RingBuf.h"
#include "Math/Utils.h"

static const uint32_t FLASHFS_ERASED_VAL = 0xffffffff;

typedef Espfc::Utils::RingBuf<uint8_t, FLASHFS_WRITE_BUFFER_SIZE> BufferType;
static BufferType buff;

static FlashfsRuntime flashfs;

const FlashfsRuntime * flashfsGetRuntime()
{
    return &flashfs;
}

static uint32_t IRAM_ATTR flashfsJournalAddress(size_t index)
{
    uint32_t base = reinterpret_cast<const esp_partition_t*>(flashfs.partition)->size - FLASHFS_JOURNAL_SIZE;
    return base + (index * sizeof(FlashfsJournalItem));
}

void flashfsJournalLoad(FlashfsJournalItem * data, size_t start, size_t num)
{
    size_t size = num * sizeof(FlashfsJournalItem);
    flashfsReadAbs(flashfsJournalAddress(start), (uint8_t*)data, size);
}

static uint32_t flashfsLogLoad()
{
    if(!flashfs.partition) return 0;

    uint32_t address = 0;
    flashfsJournalLoad(flashfs.journal, 0, FLASHFS_JOURNAL_ITEMS);
    for(size_t i = 0; i < FLASHFS_JOURNAL_ITEMS; i++)
    {
        const auto& it = flashfs.journal[i];
        if(it.logEnd == FLASHFS_ERASED_VAL) break;
        address = it.logEnd;
        flashfs.journalIdx++;
    }
    return address;
}

static void IRAM_ATTR flashfsLogBegin()
{
    uint32_t beginAddr = flashfs.address;
    size_t idx = flashfs.journalIdx;

    flashfs.journal[idx].logBegin = beginAddr;

    if(!flashfs.partition) return;

    size_t address = flashfsJournalAddress(idx);

    flashfsWriteAbs(address, (uint8_t*)&beginAddr, sizeof(uint32_t));
}

static bool IRAM_ATTR flashfsLogStarted()
{
    return flashfs.journal[flashfs.journalIdx].logBegin != FLASHFS_ERASED_VAL;
}

static void IRAM_ATTR flashfsLogEnd()
{
    uint32_t endAddr = flashfs.address;
    size_t idx = flashfs.journalIdx;

    flashfs.journal[idx].logEnd = endAddr;

    flashfs.journalIdx++;

    if(!flashfs.partition) return;

    size_t address = flashfsJournalAddress(idx) + sizeof(uint32_t);

    flashfsWriteAbs(address, (uint8_t*)&endAddr, sizeof(uint32_t));
}

int flashfsInit(void)
{
    flashfs.partition = Espfc::Device::FlashDevice::findPartition();
    if(!flashfs.partition) return 0;

    flashfs.buffer = (void*)&buff;
    flashfs.journalIdx = 0;
    flashfs.address = flashfsLogLoad();
    return 1;
}

bool IRAM_ATTR flashfsFlushAsync(bool force)
{
    if(!flashfsLogStarted()) flashfsLogBegin();

    auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
    const size_t size = buffer->size();
    if(flashfs.partition && size > 0)
    {
        //uint32_t newAddress = force ? (flashfs.address + size) : Espfc::Math::alignAddressToWrite(flashfs.address, size, FLASHFS_FLUSH_BUFFER_SIZE);
        //size_t toWrite = newAddress - flashfs.address;
        uint8_t tmp[FLASHFS_FLUSH_BUFFER_SIZE];
        size_t chunks = (size / FLASHFS_FLUSH_BUFFER_SIZE) + 1;
        while(chunks--)
        {
            size_t len = buffer->pop(tmp, FLASHFS_FLUSH_BUFFER_SIZE);
            flashfsWriteAbs(flashfs.address, tmp, len);
            flashfs.address += len;
        }
        return true;
    }
    return false;
}

void IRAM_ATTR flashfsWriteByte(uint8_t byte)
{
    if(flashfsIsEOF()) return;

    auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
    buffer->push(byte);
    const size_t size = buffer->size();

    if(flashfs.address + size >= flashfsGetSize())
    {
        flashfsClose();
        return;
    }

    if((flashfs.address + size) % SPI_FLASH_SEC_SIZE == 0)
    {
        flashfsFlushAsync(true);
        return;
    }

    if(size >= FLASHFS_FLUSH_BUFFER_SIZE)
    {
        flashfsFlushAsync(false);
    }
}

void IRAM_ATTR flashfsWrite(const uint8_t *data, unsigned int len, bool sync)
{
    (void)sync;
    while(len)
    {
        flashfsWriteByte(*data);
        data++;
        len--;
    }
}

void IRAM_ATTR flashfsWriteAbs(uint32_t address, const uint8_t *data, unsigned int len)
{
    if(!flashfs.partition) return;
    const auto p = reinterpret_cast<const esp_partition_t*>(flashfs.partition);
    esp_partition_write_raw(p, address, data, len);
}

int flashfsReadAbs(uint32_t address, uint8_t *data, unsigned int len)
{
    if(!flashfs.partition) return 0;

    const auto p = reinterpret_cast<const esp_partition_t*>(flashfs.partition);
    len = std::min((uint32_t)len, p->size - address);
    if(esp_partition_read_raw(p, address, data, len) == ESP_OK)
    {
        return len;
    }
    return 0;
}

bool IRAM_ATTR flashfsIsSupported(void)
{
    return (bool)flashfs.partition;
}

bool IRAM_ATTR flashfsIsReady(void)
{
    return flashfsIsSupported();
}

bool IRAM_ATTR flashfsIsEOF(void)
{
    return !flashfs.partition || flashfs.journalIdx >= FLASHFS_JOURNAL_ITEMS || flashfs.address >= flashfsGetSize();
}

void flashfsEraseCompletely(void)
{
    const auto p = reinterpret_cast<const esp_partition_t*>(flashfs.partition);
    esp_partition_erase_range(p, 0, p->size);
    flashfsInit();
}

void IRAM_ATTR flashfsClose(void)
{
    flashfsFlushAsync(true);
    flashfsLogEnd();
}

uint32_t IRAM_ATTR flashfsGetWriteBufferFreeSpace(void)
{
    auto buffer = reinterpret_cast<BufferType*>(flashfs.buffer);
    return buffer->available();
}

uint32_t IRAM_ATTR flashfsGetWriteBufferSize(void)
{
    return FLASHFS_WRITE_BUFFER_SIZE;
}

uint32_t IRAM_ATTR flashfsGetSize(void)
{
    const auto p = reinterpret_cast<const esp_partition_t*>(flashfs.partition);
    return p ? p->size - FLASHFS_JOURNAL_SIZE : 0;
}

uint32_t flashfsGetOffset(void)
{
    return flashfs.address;
}

uint32_t flashfsGetSectors(void)
{
    return flashfsGetSize() / SPI_FLASH_SEC_SIZE;
}

#endif
