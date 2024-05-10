
#include "InputCRSF.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Device {

using namespace Espfc::Rc;

InputCRSF::InputCRSF(): _serial(NULL), _state(CRSF_ADDR), _idx(0), _new_data(false) {}

int InputCRSF::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  for(size_t i = 0; i < CRSF_FRAME_SIZE_MAX; i++)
  {
    _frame.data[i] = 0;
    if(i < CHANNELS) _channels[i] = 0;
  }
  return 1;
}

InputStatus FAST_CODE_ATTR InputCRSF::update()
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
      parse(_frame, buff[i++]);
    }
  }

  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }

  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputCRSF::get(uint8_t i) const
{
  return _channels[i];
}

void FAST_CODE_ATTR InputCRSF::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputCRSF::getChannelCount() const { return CHANNELS; }

bool InputCRSF::needAverage() const { return false; }


void FAST_CODE_ATTR InputCRSF::parse(CrsfFrame& frame, int d)
{
  uint8_t c = (uint8_t)(d & 0xff);
  switch(_state)
  {
    case CRSF_ADDR:
      if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER)
      {
        frame.data[_idx++] = c;
        _state = CRSF_SIZE;
      }
      break;
    case CRSF_SIZE:
      if(c > 3 && c <= CRSF_PAYLOAD_SIZE_MAX)
      {
        frame.data[_idx++] = c;
        _state = CRSF_TYPE;
      } else {
        reset();
      }
      break;
    case CRSF_TYPE:
      if(c == CRSF_FRAMETYPE_RC_CHANNELS_PACKED || c == CRSF_FRAMETYPE_LINK_STATISTICS)
      {
        frame.data[_idx++] = c;
        _state = CRSF_DATA;
      } else {
        reset();
      }
      break;
    case CRSF_DATA:
      frame.data[_idx++] = c;
      if(_idx > frame.message.size) // _idx is incremented here and operator > accounts as size - 2
      {
        _state = CRSF_CRC;
      }
      break;
    case CRSF_CRC:
      frame.data[_idx++] = c;
      reset();
      uint8_t crc = Crsf::crc(frame);
      if(c == crc) {
        apply(frame);
      }
      break;
    }
}

void FAST_CODE_ATTR InputCRSF::reset()
{
  _state = CRSF_ADDR;
  _idx = 0;
}

void FAST_CODE_ATTR InputCRSF::apply(const CrsfFrame& frame)
{
  switch (frame.message.type)
  {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      applyChannels(frame);
      break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
      applyLinkStats(frame);
      break;

    default:
      break;
  }
}

void FAST_CODE_ATTR InputCRSF::applyLinkStats(const CrsfFrame f)
{
  const CrsfLinkStats* frame = reinterpret_cast<const CrsfLinkStats*>(f.message.payload);
  (void)frame;
  // TODO:
}

void FAST_CODE_ATTR InputCRSF::applyChannels(const CrsfFrame f)
{
  const CrsfData* frame = reinterpret_cast<const CrsfData*>(f.message.payload);
  Crsf::decodeRcDataShift8(_channels, frame);
  //Crsf::decodeRcData(_channels, frame);
  _new_data = true;
}

}

}

