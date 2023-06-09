#ifndef _INPUT_DEVICE_INPUT_CRSF_H_
#define _INPUT_DEVICE_INPUT_CRSF_H_

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Rc/Crsf.h"

// https://github.com/CapnBry/CRServoF/blob/master/lib/CrsfSerial/crsf_protocol.h
// https://github.com/AlessioMorale/crsf_parser/tree/master
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c

namespace Espfc {

namespace Device {

using namespace Espfc::Rc;

class InputCRSF: public InputDevice
{
  public:
    enum CrsfState {
      CRSF_ADDR,
      CRSF_SIZE,
      CRSF_TYPE,
      CRSF_DATA,
      CRSF_CRC
    };

    InputCRSF(): _serial(NULL), _state(CRSF_ADDR), _idx(0), _new_data(false) {}

    int begin(Device::SerialDevice * serial)
    {
      _serial = serial;
      for(size_t i = 0; i < CRSF_FRAME_SIZE_MAX; i++)
      {
        _frame.data[i] = 0;
        if(i < CHANNELS) _channels[i] = 0;
      }
      return 1;
    }

    InputStatus update() override
    {
      if(!_serial) return INPUT_IDLE;

      size_t len = _serial->available();
      while(len--)
      {
        parse(_frame, _serial->read());
      }

      if(_new_data)
      {
        _new_data = false;
        return INPUT_RECEIVED;
      }

      return INPUT_IDLE;
    }

    uint16_t get(uint8_t i) const override
    {
      return _channels[i];
    }

    size_t getChannelCount() const override { return CHANNELS; }

    bool needAverage() const override { return false; }

    void print(char c) const
    {
      //Serial.write(c);
    }

    void parse(CrsfFrame& frame, int d)
    {
      uint8_t c = (uint8_t)(d & 0xff);
      //print(c);
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

  private:
    void reset()
    {
      _state = CRSF_ADDR;
      _idx = 0;
    }

    void apply(const CrsfFrame& frame)
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

    void applyLinkStats(const CrsfFrame f)
    {
      const CrsfLinkStats* frame = reinterpret_cast<const CrsfLinkStats*>(f.message.payload);
      (void)frame;
      // TODO:
    }

    void applyChannels(const CrsfFrame f)
    {
      const CrsfData* frame = reinterpret_cast<const CrsfData*>(f.message.payload);
      Crsf::decodeRcData(_channels, frame);
      _new_data = true;
    }

    static const size_t CHANNELS = 16;

    Device::SerialDevice * _serial;
    CrsfState _state;
    uint8_t _idx;
    bool _new_data;
    CrsfFrame _frame;
    uint16_t _channels[CHANNELS];
};

}

}

#endif
