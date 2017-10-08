#ifndef _ESPFC_MSP_H_
#define _ESPFC_MSP_H_

#include <stdlib.h>
#include <stdint.h>
#include "Arduino.h"

#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1  // increment when major changes are made
#define API_VERSION_MINOR                   36 // increment after a release, to set the version for all changes to go into the following release (if no changes to MSP are made between the releases, this can be reverted before the release)
#define API_VERSION_LENGTH                  2

#define MSP_API_VERSION                 1    //out message
#define MSP_FC_VARIANT                  2    //out message
#define MSP_FC_VERSION                  3    //out message
#define MSP_BOARD_INFO                  4    //out message
#define MSP_BUILD_INFO                  5    //out message

#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_VERSION_MAJOR            3  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            2  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#define BETAFLIGHT_IDENTIFIER "BTFL"
#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"

static const char * flightControllerIdentifier = BETAFLIGHT_IDENTIFIER;
static const char * boardIdentifier = "ESPF";

#include "Model.h"

namespace Espfc {

static const size_t MSP_BUF_SIZE = 192;

class Msp
{
  public:
    enum MspState
    {
      STATE_IDLE,
      STATE_HEADER_START,
      STATE_HEADER_M,
      STATE_HEADER_ARROW,
      STATE_HEADER_SIZE,
      STATE_HEADER_CMD,
      STATE_RECEIVED
    };

    enum MspType
    {
      TYPE_CMD,
      TYPE_REPLY
    };

    enum MspCmd
    {
      MSP_IDENT                = 100,
  	  MSP_STATUS               = 101,
  	  MSP_RAW_IMU              = 102,
  	  MSP_SERVO                = 103,
  	  MSP_MOTOR                = 104,
    };

    class MspMessage
    {
      public:
        MspMessage() {}
        MspState state;
        MspType dir;
        uint8_t cmd;
        uint8_t expected;
        uint8_t received;
        uint8_t buffer[MSP_BUF_SIZE];
        uint8_t checksum;
    };

    class MspResponse {
      public:
        MspResponse(): len(0) {}
        int16_t cmd;
        int16_t result;
        uint8_t direction;
        uint8_t len;
        uint8_t data[MSP_BUF_SIZE];

        void writeData(const char * v, int size)
        {
          while(size-- > 0) writeU8(*v++);
        }

        void writeU8(uint8_t v)
        {
          data[len++] = v;
        }

        void writeU16(uint16_t v)
        {
          writeU8(v >> 0);
          writeU8(v >> 8);
        }

        void writeU32(uint32_t v)
        {
          writeU8(v >> 0);
          writeU8(v >> 8);
          writeU8(v >> 16);
          writeU8(v >> 24);
        }
    };


    Msp(Model& model): _model(model) {}

    bool process(char c, Stream& s)
    {
      switch(_msg.state)
      {
        case STATE_IDLE:               // sync char 1 '$'
          if(c == '$') _msg.state = STATE_HEADER_START;
          break;

        case STATE_HEADER_START:       // sync char 2 'M'
            if(c == 'M') _msg.state = STATE_HEADER_M;
            else _msg.state = STATE_IDLE;
            break;

        case STATE_HEADER_M:               // direction (should be >)
          switch(c)
          {
            case '>':
              _msg.dir = TYPE_REPLY;
              _msg.state = STATE_HEADER_ARROW;
              break;
            case '<':
              _msg.dir = TYPE_CMD;
              _msg.state = STATE_HEADER_ARROW;
              break;
            default:
              _msg.state = STATE_IDLE;
          }
          break;
        case STATE_HEADER_ARROW:
          if (c > MSP_BUF_SIZE) _msg.state = STATE_IDLE;
          else
          {
            _msg.expected = c;
            _msg.received = 0;
            _msg.checksum = 0;
            _msg.checksum ^= c;
            _msg.state = STATE_HEADER_SIZE;
          }
          break;

        case STATE_HEADER_SIZE:
          _msg.cmd = c;
          _msg.checksum ^= c;
          _msg.state = STATE_HEADER_CMD;
          break;

        case STATE_HEADER_CMD:
          if(_msg.received < _msg.expected)
          {
            _msg.checksum ^= c;
            _msg.buffer[_msg.received++] = c;
          }
          else if(_msg.received >= _msg.expected)
          {
            _msg.state = _msg.checksum == c ? STATE_RECEIVED : STATE_IDLE;
          }

        default:
          break;
      }

      if(_msg.state == STATE_RECEIVED)
      {
        _msg.state = STATE_IDLE;
        switch(_msg.dir)
        {
          case TYPE_CMD:
            processCommand(_msg);
            break;
          case TYPE_REPLY:
            processReply(_msg, s);
            break;
        }
        return true;
      }

      return _msg.state != STATE_IDLE;
    }

    void processCommand(MspMessage& m)
    {

    }

    void processReply(MspMessage& m, Stream& s)
    {
      MspResponse r;
      r.cmd = m.cmd;
      switch(m.cmd)
      {
        case MSP_API_VERSION:
          r.writeU8(MSP_PROTOCOL_VERSION);
          r.writeU8(API_VERSION_MAJOR);
          r.writeU8(API_VERSION_MINOR);
          break;

        case MSP_FC_VARIANT:
          r.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
          break;

        case MSP_FC_VERSION:
          r.writeU8(FC_VERSION_MAJOR);
          r.writeU8(FC_VERSION_MINOR);
          r.writeU8(FC_VERSION_PATCH_LEVEL);
          break;

        case MSP_BOARD_INFO:
          r.writeData(boardIdentifier, BOARD_IDENTIFIER_LENGTH);
          r.writeU16(0); // No other build targets currently have hardware revision detection.
          r.writeU8(0);  // 0 == FC
          break;

        case MSP_BUILD_INFO:
          r.writeData(buildDate, BUILD_DATE_LENGTH);
          r.writeData(buildTime, BUILD_TIME_LENGTH);
          r.writeData(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
          break;

        default:
          break;
      }
      sendResponse(r, s);
    }

    void sendResponse(MspResponse& r, Stream& s)
    {
      uint8_t hdr[5] = { '$', 'M', '>' };
      hdr[3] = r.len;
      hdr[4] = r.cmd;
      uint8_t checksum = crc(0, &hdr[3], 2);
      s.write(hdr, 5);
      if(r.len > 0)
      {
        s.write(r.data, r.len);
        checksum = crc(checksum, r.data, r.len);
      }
      s.write(checksum);
    }

    uint8_t crc(uint8_t checksum, const uint8_t *data, int len)
    {
      while (len-- > 0)
      {
        checksum ^= *data++;
      }
      return checksum;
    }

  private:
    Model& _model;
    MspMessage _msg;
};

}

#endif
