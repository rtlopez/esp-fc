#include <Arduino.h>
#include <platform.h>
#include "Device/SerialDevice.h"
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

void FAST_CODE_ATTR serialWrite(serialPort_t * instance, uint8_t ch)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(dev) dev->write(ch);
}

uint32_t FAST_CODE_ATTR serialRxBytesWaiting(serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(!dev) return 0;
  return dev->available();
}

int FAST_CODE_ATTR serialRead(serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(dev) return dev->read();
  return -1;
}

uint32_t FAST_CODE_ATTR serialTxBytesFree(const serialPort_t * instance)
{
  Espfc::Device::SerialDevice * dev = (Espfc::Device::SerialDevice *)instance->espfcDevice;
  if(!dev) return 0;
  return dev->availableForWrite();
}

bool FAST_CODE_ATTR isSerialTransmitBufferEmpty(const serialPort_t * instance)
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
