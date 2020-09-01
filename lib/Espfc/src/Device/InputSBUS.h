#ifndef _INPUT_DEVICE_INPUT_SBUS_H_
#define _INPUT_DEVICE_INPUT_SBUS_H_

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Math/Utils.h"

namespace Espfc {

namespace Device {

struct SbusData
{
  uint8_t syncByte;
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
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
  uint8_t flags;
  /**
   * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
   * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
   * and https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
   */
  uint8_t endByte;
} __attribute__ ((__packed__));

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

class InputSBUS: public InputDevice
{
  public:
    enum SbusState {
      SBUS_START,
      SBUS_DATA,
      SBUS_END
    };

    InputSBUS(): _serial(NULL), _state(SBUS_START), _idx(0), _new_data(false) {}

    int begin(Device::SerialDevice * serial)
    {
      _serial = serial;
      for(size_t i = 0; i < SBUS_FRAME_SIZE; i++)
      {
        _data[i] = 0;
        if(i < CHANNELS) _channels[i] = 0;
      }
      return 1;
    }

    InputStatus update() override
    {
      if(!_serial) return INPUT_IDLE;

      if((*_serial).available() >= (int)SBUS_FRAME_SIZE)
      {
        while((*_serial).available())
        {
          parse((*_serial).read());
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

  private:
    void parse(int d)
    {
      char c = (char)(d & 0xff);
      //print(c);
      //print(checkParityEven(d));
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

    bool checkParityEven(int d) const
    {
      bool parity = (d >> 8) & 0x1;
      for(size_t i = 0; i < 8; i++)
      {
        if(d & (1 << i)) parity = !parity;
      }
      return !parity;
    }

    void apply()
    {
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
      _new_data = true;
    }

    inline uint16_t convert(int v)
    {
      return Math::clamp(((v * 5) / 8) + 880, 800, 2200);
    }

    const static size_t SBUS_FRAME_SIZE = sizeof(SbusData);

    Device::SerialDevice * _serial;
    SbusState _state;
    uint8_t _idx = 0;
    bool _new_data;

    static const size_t CHANNELS = 16;

    uint8_t _data[SBUS_FRAME_SIZE];
    uint16_t _channels[CHANNELS];
    uint8_t _flags;
};

}

}

#endif
