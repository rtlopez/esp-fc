#include "InputIBUS.hpp"
#include "Utils/MemoryHelper.h"
#include <algorithm>

namespace Espfc::Device
{

InputIBUS::InputIBUS() : _serial(NULL), _state(IBUS_LENGTH), _idx(0), _new_data(false) {}

int InputIBUS::begin(Device::SerialDevice *serial)
{
  _serial = serial;

  std::fill_n((uint8_t*)&_data, IBUS_FRAME_SIZE, 0);
  std::fill_n(_channels, CHANNELS, 0);

  return 1;
}

InputStatus FAST_CODE_ATTR InputIBUS::update()
{
  if (!_serial) return INPUT_IDLE;

  uint8_t buff[64] = {0};
  size_t len = std::min((size_t)_serial->available(), (size_t)sizeof(buff));

  if (len)
  {
    _serial->readMany(buff, len);
    uint8_t* ptr = buff;
    while(len--)
    {
      parse(_data, *ptr++);
    }
  }

  if (_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
}

void FAST_CODE_ATTR InputIBUS::parse(IBusData& frameData, int d)
{
  uint8_t* data = reinterpret_cast<uint8_t*>(&frameData);
  uint8_t c = d & 0xff;
  switch(_state)
  {
    case IBUS_LENGTH:
      if(c == IBUS_FRAME_SIZE)
      {
        data[_idx++] = c;
        _state = IBUS_CMD;
      }
      break;
    case IBUS_CMD:
      if(c == IBUS_COMMAND)
      {
        data[_idx++] = c;
        _state = IBUS_DATA;
      }
      else
      {
        _state = IBUS_LENGTH;
      }
      break;
    case IBUS_DATA:
      data[_idx] = c;
      if(++_idx >= IBUS_FRAME_SIZE - 2)
      {
        _state = IBUS_CRC_LO;
      }
      break;
    case IBUS_CRC_LO:
      data[_idx++] = c;
      _state = IBUS_CRC_HI;
      break;
    case IBUS_CRC_HI:
      data[_idx++] = c;
      uint16_t csum = 0xffff;
      for(size_t i = 0; i < IBUS_FRAME_SIZE - 2; i++)
      {
        csum -= data[i];
      }
      if(frameData.checksum == csum) apply(frameData);
      _state = IBUS_LENGTH;
      _idx = 0;
      break;
    }
}

void FAST_CODE_ATTR InputIBUS::apply(IBusData& data)
{
  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = data.ch[i];
  }
  _new_data = true;
}

uint16_t FAST_CODE_ATTR InputIBUS::get(uint8_t i) const
{
  return _channels[i];
}

void FAST_CODE_ATTR InputIBUS::get(uint16_t *data, size_t len) const
{
  const uint16_t *src = _channels;
  while (len--)
  {
    *data++ = *src++;
  }
}

size_t InputIBUS::getChannelCount() const { return CHANNELS; }

bool InputIBUS::needAverage() const { return false; }

}