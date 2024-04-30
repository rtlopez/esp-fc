#include <Arduino.h>
#include "platform.h"
#include "Device/SerialDevice.h"
#include "Device/FlashDevice.h"
#include "EscDriver.h"
#include "Hal/Gpio.h"
#include "Utils/MemoryHelper.h"

int IORead(IO_t pin)
{
    return Espfc::Hal::Gpio::digitalRead(pin);
}

void IOConfigGPIO(IO_t pin, uint8_t mode)
{
    switch(mode) {
        case IOCFG_IPU:
            ::pinMode(pin, INPUT_PULLUP);
            break;
        case IOCFG_OUT_PP:
        case IOCFG_AF_PP:
            ::pinMode(pin, OUTPUT);
            break;
    }
}

void IOHi(IO_t pin)
{
    Espfc::Hal::Gpio::digitalWrite(pin, HIGH);
}

void IOLo(IO_t pin)
{
    Espfc::Hal::Gpio::digitalWrite(pin, LOW);
}

static serialPort_t _sp[2] = {{
    .txBufferSize = 128
}, {
    .txBufferSize = 128
}};

static serialPortConfig_t _spc = {
    .identifier = SERIAL_PORT_USART1,
    .blackbox_baudrateIndex = 5,
};

void serialDeviceInit(void * serial, size_t index)
{
  if(index > 1) return;
  _sp[index].espfcDevice = serial;
}

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    return NULL;
}

void mspSerialReleasePortIfAllocated(serialPort_t *serialPort)
{
    UNUSED(serialPort);
}

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    return &_spc;
}

serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudrate, portMode_e mode, portOptions_e options)
{
    return &_sp[0];
}

serialPort_t *getSerialPort()
{
    return &_sp[0];
}

void closeSerialPort(serialPort_t *serialPort)
{
    UNUSED(serialPort);
}

void mspSerialAllocatePorts(void)
{
}

portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function)
{
    return PORTSHARING_UNUSED;
}

void serialBeginWrite(serialPort_t * instance)
{
}

void serialEndWrite(serialPort_t * instance)
{
}

void serialWrite(serialPort_t * instance, uint8_t ch)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(dev) dev->write(ch);
}

uint32_t serialRxBytesWaiting(serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(!dev) return 0;
  return dev->available();
}

int serialRead(serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(dev) return dev->read();
  return -1;
}

uint32_t serialTxBytesFree(const serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(!dev) return 0;
  return dev->availableForWrite();
}

bool isSerialTransmitBufferEmpty(const serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(!dev) return 0;
  return dev->isTxFifoEmpty();
}

static size_t _motorCount = 0;
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

pwmOutputPort_t * pwmGetMotors(void)
{
    return motors;
}

static EscDriver * escDriver;

void motorInitEscDevice(void * driver)
{
    escDriver = static_cast<EscDriver *>(driver);
    memset(&motors, 0, sizeof(pwmOutputPort_t));
    if(!driver) return;

    for(size_t i = 0; i < MAX_SUPPORTED_MOTORS; i++)
    {
        int pin = escDriver->pin(i);
        if(pin == -1) continue;
        motors[i].enabled = true;
        motors[i].io = pin;
        _motorCount++;
    }
}

void motorDisable(void)
{
    if(escDriver) escDriver->end();
}

void motorEnable(void)
{
}

uint8_t getMotorCount()
{
    return _motorCount;
}

void beeper(int mode)
{

}

// FLASHFS START
static const uint32_t FLASHFS_ERASED_VAL = 0xffffffff;

static FlashfsRuntime flashfs;

const FlashfsRuntime * flashfsGetRuntime()
{
    return &flashfs;
}

static uint32_t IRAM_ATTR flashfsJournalAddress(size_t index)
{
    return flashfs.partition->size - FLASHFS_JOURNAL_SIZE + index * sizeof(FlashfsJournalItem);
}

void flashfsJournalLoad(FlashfsJournalItem * data, size_t num)
{
    size_t size = num * sizeof(FlashfsJournalItem);
    flashfsReadAbs(flashfsJournalAddress(0), (uint8_t*)data, size);
}

static uint32_t flashfsLogLoad()
{
    if(!flashfs.partition) return 0;

    uint32_t address = 0;
    flashfsJournalLoad(flashfs.journal, FLASHFS_JOURNAL_ITEMS);
    for(size_t i = 0; i < FLASHFS_JOURNAL_ITEMS; i++)
    {
        const auto& it = flashfs.journal[i];
        if(it.logEnd == FLASHFS_ERASED_VAL) break;
        address = it.logEnd;
        flashfs.journalIdx++;
    }
    flashfs.journal[flashfs.journalIdx] = {0, 0};
    return address;
}

static void IRAM_ATTR flashfsLogBegin()
{
    flashfs.journal[flashfs.journalIdx].logBegin = flashfs.address;

    if(!flashfs.partition) return;

    size_t address = flashfsJournalAddress(flashfs.journalIdx);

    flashfsWriteAbs(address, (uint8_t*)&flashfs.address, sizeof(uint32_t));
}

static void IRAM_ATTR flashfsLogEnd()
{
    flashfs.journal[flashfs.journalIdx].logEnd = flashfs.address;
    size_t idx = flashfs.journalIdx;
    flashfs.journalIdx++;
    flashfs.journal[flashfs.journalIdx] = {0, 0};

    if(!flashfs.partition) return;

    size_t address = flashfsJournalAddress(idx);

    flashfsWriteAbs(address + sizeof(uint32_t), (uint8_t*)&flashfs.address, sizeof(uint32_t));
}

int flashfsInit(void)
{
    flashfs.partition = Espfc::Device::FlashDevice::findPartition();
    if(!flashfs.partition) return 0;

    flashfs.bufferIdx = 0;
    flashfs.journalIdx = 0;
    flashfs.address = flashfsLogLoad();
    return 1;
}

void IRAM_ATTR flashfsWriteByte(uint8_t byte)
{
    if(flashfsIsEOF()) return;
    flashfs.buffer[flashfs.bufferIdx] = byte;
    flashfs.bufferIdx++;
    if(flashfs.address + flashfs.bufferIdx >= flashfsGetSize())
    {
        flashfsClose();
        return;
    }
    if(flashfs.bufferIdx >= FLASHFS_WRITE_BUFFER_SIZE)
    {
        flashfsFlushAsync(true);
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
    esp_partition_write_raw(flashfs.partition, address, data, len);
}

int flashfsReadAbs(uint32_t address, uint8_t *data, unsigned int len)
{
    if(!flashfs.partition) return 0;

    len = std::min(len, flashfs.partition->size - address);
    if(esp_partition_read_raw(flashfs.partition, address, data, len) == ESP_OK)
    {
        return len;
    }
    return 0;
}

bool IRAM_ATTR flashfsFlushAsync(bool force)
{
    (void)force;
    if(flashfs.partition && flashfs.bufferIdx)
    {
        // write to journal before first flush
        if(!flashfs.journal[flashfs.journalIdx].logBegin)
        {
            flashfsLogBegin();
        }
        flashfsWriteAbs(flashfs.address, flashfs.buffer, flashfs.bufferIdx);
        flashfs.address += flashfs.bufferIdx;
        flashfs.bufferIdx = 0;
    }
    return true;
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
    esp_partition_erase_range(flashfs.partition, 0, flashfs.partition->size);
    flashfsInit();
}

void IRAM_ATTR flashfsClose(void)
{
    flashfsFlushAsync(true);
    flashfsLogEnd();
}

uint32_t IRAM_ATTR flashfsGetWriteBufferFreeSpace(void)
{
    return FLASHFS_WRITE_BUFFER_SIZE - flashfs.bufferIdx;
}

uint32_t IRAM_ATTR flashfsGetWriteBufferSize(void)
{
    return FLASHFS_WRITE_BUFFER_SIZE;
}

uint32_t IRAM_ATTR flashfsGetSize(void)
{
    //return partition ? partition->size : 0;
    return flashfs.partition ? (flashfs.partition->size - FLASHFS_JOURNAL_SIZE) : 0;
}

uint32_t flashfsGetOffset(void)
{
    return flashfs.address;
}

uint32_t flashfsGetSectors(void)
{
    return flashfsGetSize() / SPI_FLASH_SEC_SIZE;
}