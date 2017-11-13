#ifndef _INPUT_SBUS_H_
#define _INPUT_SBUS_H_

#include <Arduino.h>
#include "SerialDevice.h"
#include "InputDevice.h"

namespace Espfc {

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

class InputSBUS: public InputDevice
{
  public:
    enum SbusState {
      SBUS_START,
      SBUS_DATA,
      SBUS_END
    };

    InputSBUS(): _serial(NULL), _state(SBUS_START), _idx(0), _new_data(false) {}

    int begin(SerialDevice * serial)
    {
      _serial = serial;
      for(size_t i = 0; i < DATA_SIZE; i++)
      {
        _data[i] = 0;
        if(i < 16) _channels[i] = 0;
      }
      return 1;
    }

    InputStatus update() override
    {
      if(!_serial) return INPUT_IDLE;

      while((*_serial).available())
      {
        parse((*_serial).read());
      }

      if(_new_data)
      {
        _new_data = false;
        return INPUT_RECEIVED;
      }
      //TODO: check failsafe
      return INPUT_IDLE;
    }

    uint16_t get(uint8_t i) const override
    {
      return map(_channels[i], 220, 1820, 1000, 2000);
    }

  private:
    void parse(uint8_t c)
    {
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
          _data[_idx++] = c;
          if(_idx >= DATA_SIZE - 1)
          {
            _state = SBUS_END;
          }
          break;
        case SBUS_END:
          if(c == 0x00)
          {
            _data[_idx++] = c;
            _state = SBUS_START;
            _idx = 0;

            // convert channels
            _channels[0]  = ((_data[1]       | _data[2]  << 8) & 0x07FF);
            _channels[1]  = ((_data[2]  >> 3 | _data[3]  << 5) & 0x07FF);
            _channels[2]  = ((_data[3]  >> 6 | _data[4]  << 2 | _data[5] << 10) & 0x07FF);
            _channels[3]  = ((_data[5]  >> 1 | _data[6]  << 7) & 0x07FF);
            _channels[4]  = ((_data[6]  >> 4 | _data[7]  << 4) & 0x07FF);
            _channels[5]  = ((_data[7]  >> 7 | _data[8]  << 1 | _data[9] << 9) & 0x07FF);
            _channels[6]  = ((_data[9]  >> 2 | _data[10] << 6) & 0x07FF);
            _channels[7]  = ((_data[10] >> 5 | _data[11] << 3) & 0x07FF);
            _channels[8]  = ((_data[12]      | _data[13] << 8) & 0x07FF);
            _channels[9]  = ((_data[13] >> 3 | _data[14] << 5)  & 0x07FF);
            _channels[10] = ((_data[14] >> 6 | _data[15] << 2| _data[16] << 10) & 0x07FF);
            _channels[11] = ((_data[16] >> 1 | _data[17] << 7) & 0x07FF);
            _channels[12] = ((_data[17] >> 4 | _data[18] << 4) & 0x07FF);
            _channels[13] = ((_data[18] >> 7 | _data[19] << 1| _data[20] << 9)  & 0x07FF);
            _channels[14] = ((_data[20] >> 2 | _data[21] << 6) & 0x07FF);
            _channels[15] = ((_data[21] >> 5 | _data[22] << 3) & 0x07FF);
            //_channels[16] = ((_data[23]));

            _new_data = true;
          }
        }
    }

    const static size_t DATA_SIZE = 25;

    SerialDevice * _serial;
    SbusState _state;
    uint8_t _idx = 0;
    bool _new_data;

    uint8_t _data[DATA_SIZE];
    uint16_t _channels[16];
};

}

#endif
