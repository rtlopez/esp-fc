#include <Arduino.h>
#include "platform.h"
#include "Device/SerialDevice.h"

int IORead(IO_t pin)
{
    return digitalRead(pin);
}

void IOConfigGPIO(IO_t pin, uint8_t mode)
{
    pinMode(pin, mode);
}

void IOHi(IO_t pin)
{
    digitalWrite(pin, HIGH);
}

void IOLo(IO_t pin)
{
    digitalWrite(pin, LOW);
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
