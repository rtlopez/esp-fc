#include "InputSBUS.h"
#include "Math/Utils.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Device {

InputSBUS::InputSBUS(): _serial(NULL), _state(SBUS_START), _idx(0), _new_data(false) {}

int InputSBUS::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  for(size_t i = 0; i < SBUS_FRAME_SIZE; i++)
  {
    _data[i] = 0;
    if(i < CHANNELS) _channels[i] = 0;
  }
  return 1;
}

InputStatus FAST_CODE_ATTR InputSBUS::update()
{
  if(!_serial) return INPUT_IDLE;

  size_t len = _serial->available();
  if(len)
  {
    uint8_t buff[64] = {0};
    len = std::min(len, sizeof(buff));
    _serial->readMany(buff, len);
    size_t i = 0;
    while(i < len)
    {
      parse(buff[i++]);
    }
  }

  if(_new_data)
  {
    _new_data = false;
    if(_flags & SBUS_FLAG_FAILSAFE_ACTIVE) return INPUT_FAILSAFE;
    if(_flags & SBUS_FLAG_SIGNAL_LOSS) return INPUT_LOST;
    return INPUT_RECEIVED;
  }

  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputSBUS::get(uint8_t i) const
{
  return _channels[i];
}

void FAST_CODE_ATTR InputSBUS::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputSBUS::getChannelCount() const { return CHANNELS; }

bool InputSBUS::needAverage() const { return false; }

void FAST_CODE_ATTR InputSBUS::parse(int d)
{
  char c = (char)(d & 0xff);
  switch(_state)
  {
    case SBUS_START:
      if(c == 0x0F)
      {
        _data[_idx++] = c;
        _state = SBUS_DATA;
      }
      break;
    case SBUS_DATA:
      _data[_idx] = c;
      if(++_idx >= SBUS_FRAME_SIZE - 1)
      {
        _state = SBUS_END;
      }
      break;
    case SBUS_END:
      _data[_idx++] = c;
      _state = SBUS_START;
      _idx = 0;
      apply();
      break;
    }
}

void FAST_CODE_ATTR InputSBUS::apply()
{
  /*
  const SbusData* frame = reinterpret_cast<const SbusData*>(_data);
  _channels[0]  = convert(frame->chan0);
  _channels[1]  = convert(frame->chan1);
  _channels[2]  = convert(frame->chan2);
  _channels[3]  = convert(frame->chan3);
  _channels[4]  = convert(frame->chan4);
  _channels[5]  = convert(frame->chan5);
  _channels[6]  = convert(frame->chan6);
  _channels[7]  = convert(frame->chan7);
  _channels[8]  = convert(frame->chan8);
  _channels[9]  = convert(frame->chan9);
  _channels[10] = convert(frame->chan10);
  _channels[11] = convert(frame->chan11);
  _channels[12] = convert(frame->chan12);
  _channels[13] = convert(frame->chan13);
  _channels[14] = convert(frame->chan14);
  _channels[15] = convert(frame->chan15);
  _flags = frame->flags;
  */

  // shifting is more efficient
  _channels[0]  = convert((_data[1]       | _data[2]  << 8) & 0x07FF);
  _channels[1]  = convert((_data[2]  >> 3 | _data[3]  << 5) & 0x07FF);
  _channels[2]  = convert((_data[3]  >> 6 | _data[4]  << 2 | _data[5] << 10) & 0x07FF);
  _channels[3]  = convert((_data[5]  >> 1 | _data[6]  << 7) & 0x07FF);
  _channels[4]  = convert((_data[6]  >> 4 | _data[7]  << 4) & 0x07FF);
  _channels[5]  = convert((_data[7]  >> 7 | _data[8]  << 1 | _data[9] <<  9) & 0x07FF);
  _channels[6]  = convert((_data[9]  >> 2 | _data[10] << 6) & 0x07FF);
  _channels[7]  = convert((_data[10] >> 5 | _data[11] << 3) & 0x07FF);
  _channels[8]  = convert((_data[12]      | _data[13] << 8) & 0x07FF);
  _channels[9]  = convert((_data[13] >> 3 | _data[14] << 5) & 0x07FF);
  _channels[10] = convert((_data[14] >> 6 | _data[15] << 2 | _data[16] << 10) & 0x07FF);
  _channels[11] = convert((_data[16] >> 1 | _data[17] << 7) & 0x07FF);
  _channels[12] = convert((_data[17] >> 4 | _data[18] << 4) & 0x07FF);
  _channels[13] = convert((_data[18] >> 7 | _data[19] << 1 | _data[20] <<  9) & 0x07FF);
  _channels[14] = convert((_data[20] >> 2 | _data[21] << 6) & 0x07FF);
  _channels[15] = convert((_data[21] >> 5 | _data[22] << 3) & 0x07FF);
  _flags = _data[23];

  _new_data = true;
}

uint16_t FAST_CODE_ATTR InputSBUS::convert(int v)
{
  return Math::clamp(((v * 5) / 8) + 880, 800, 2200);
}

}

}
