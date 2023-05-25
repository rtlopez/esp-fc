#ifndef _INPUT_DEVICE_INPUT_CRSF_H_
#define _INPUT_DEVICE_INPUT_CRSF_H_

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Math/Utils.h"
#include "Math/Crc.h"

// https://github.com/CapnBry/CRServoF/blob/master/lib/CrsfSerial/crsf_protocol.h
// https://github.com/AlessioMorale/crsf_parser/tree/master
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c

namespace Espfc {

namespace Device {

enum { CRSF_SYNC_BYTE = 0xC8 };
enum { CRSF_FRAME_SIZE_MAX = 64 }; // 62 bytes frame plus 2 bytes frame header(<length><type>)
enum { CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6 };

enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
};

enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
};

struct CrsfData
{
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
} __attribute__ ((__packed__));

/*
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 */
struct CrsfMessage
{
  uint8_t addr; // CrsfAddress
  uint8_t size; // counts size after this byte, so it must be the payload size + 2 (type and crc)
  uint8_t type; // CrsfFrameType
  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];
} __attribute__ ((__packed__));

union CrsfFrame
{
  CrsfMessage message;
  uint8_t data[CRSF_FRAME_SIZE_MAX];
};

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

      if((*_serial).available() >= 4)
      {
        while((*_serial).available())
        {
          parse(_frame, (*_serial).read());
        }
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
          if(c == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
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
          uint8_t crc = frameCrc(frame);
          if(c == crc) {
            apply(frame);
          }
          break;
       }
    }

    uint8_t frameCrc(const CrsfFrame& frame)
    {
      // CRC includes type and payload
      uint8_t crc = Math::crc8_dvb_s2(0, frame.message.type);
      for (int i = 0; i < frame.message.size - 2; i++) { // size includes type and crc
          crc = Math::crc8_dvb_s2(crc, frame.message.payload[i]);
      }
      return crc;
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
        
        default:
          break;
      }
    }

    void applyChannels(const CrsfFrame f)
    {
      const CrsfData* frame = reinterpret_cast<const CrsfData*>(f.message.payload);
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
      _new_data = true;
    }

    inline uint16_t convert(int v)
    {
      return Math::map(v, 172, 1811, 988, 2012);
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
