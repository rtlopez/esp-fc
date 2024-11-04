#include <algorithm>
#include "InputCRSF.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Device {

using namespace Espfc::Rc;

InputCRSF::InputCRSF(): _serial(NULL), _telemetry(NULL), _state(CRSF_ADDR), _idx(0), _new_data(false) {}

int InputCRSF::begin(Device::SerialDevice * serial, TelemetryManager * telemetry)
{
  _serial = serial;
  _telemetry = telemetry;
  _telemetry_next = micros() + TELEMETRY_INTERVAL;
  std::fill_n((uint8_t*)&_frame, sizeof(_frame), 0);
  std::fill_n(_channels, CHANNELS, 0);
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

  if(_telemetry && micros() > _telemetry_next)
  {
    _telemetry_next = micros() + TELEMETRY_INTERVAL;
    _telemetry->process(*_serial, TELEMETRY_PROTOCOL_CRSF);
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

void FAST_CODE_ATTR InputCRSF::parse(CrsfMessage& msg, int d)
{
  uint8_t *data = reinterpret_cast<uint8_t*>(&msg);
  uint8_t c = (uint8_t)(d & 0xff);
  switch(_state)
  {
    case CRSF_ADDR:
      if(c == CRSF_SYNC_BYTE)
      {
        data[_idx++] = c;
        _state = CRSF_SIZE;
      }
      break;
    case CRSF_SIZE:
      if(c > 3 && c <= CRSF_PAYLOAD_SIZE_MAX)
      {
        data[_idx++] = c;
        _state = CRSF_TYPE;
      } else {
        reset();
      }
      break;
    case CRSF_TYPE:
      if(c == CRSF_FRAMETYPE_RC_CHANNELS_PACKED || c == CRSF_FRAMETYPE_LINK_STATISTICS || c == CRSF_FRAMETYPE_MSP_REQ)
      {
        data[_idx++] = c;
        _state = CRSF_DATA;
      } else {
        reset();
      }
      break;
    case CRSF_DATA:
      data[_idx++] = c;
      if(_idx > msg.size) // _idx is incremented here and operator > accounts as size - 2
      {
        _state = CRSF_CRC;
      }
      break;
    case CRSF_CRC:
      data[_idx++] = c;
      reset();
      uint8_t crc = msg.crc();
      if(c == crc) {
        apply(msg);
      }
      break;
    }
}

void FAST_CODE_ATTR InputCRSF::reset()
{
  _state = CRSF_ADDR;
  _idx = 0;
}

void FAST_CODE_ATTR InputCRSF::apply(const CrsfMessage& msg)
{
  switch (msg.type)
  {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      applyChannels(msg);
      break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
      applyLinkStats(msg);
      break;

    case CRSF_FRAMETYPE_MSP_REQ:
      applyMspReq(msg);
      break;

    default:
      break;
  }
}

void FAST_CODE_ATTR InputCRSF::applyLinkStats(const CrsfMessage& msg)
{
  const CrsfLinkStats* stats = reinterpret_cast<const CrsfLinkStats*>(msg.payload);
  (void)stats;
  // TODO:
}

void FAST_CODE_ATTR InputCRSF::applyChannels(const CrsfMessage& msg)
{
  const CrsfData* data = reinterpret_cast<const CrsfData*>(msg.payload);
  Crsf::decodeRcDataShift8(_channels, data);
  //Crsf::decodeRcData(_channels, frame);
  _new_data = true;
}

void FAST_CODE_ATTR InputCRSF::applyMspReq(const CrsfMessage& msg)
{
  if(!_telemetry) return;

  uint8_t origin;
  Connect::MspMessage m;

  Crsf::decodeMsp(msg, m, origin);

  if(m.isCmd() && m.isReady())
  {
    _telemetry->processMsp(*_serial, TELEMETRY_PROTOCOL_CRSF, m, origin);
  }

  _telemetry_next = micros() + TELEMETRY_INTERVAL;
}

}

}

