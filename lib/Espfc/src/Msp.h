#ifndef _ESPFC_MSP_H_
#define _ESPFC_MSP_H_

#include <stdlib.h>
#include <stdint.h>
#include "Arduino.h"
#include "msp/msp_protocol.h"

#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_VERSION_MAJOR            3  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            2  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

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
        uint8_t cmd;
        uint8_t result;
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
        debugMessage(_msg);
        switch(_msg.dir)
        {
          case TYPE_CMD:
            processReply(_msg, s);
            break;
          case TYPE_REPLY:
            processReply(_msg, s);
            break;
        }
        return true;
      }

      return _msg.state != STATE_IDLE;
    }

    void debugMessage(const MspMessage& m)
    {
      if(!debugSkip(m.cmd)) return;

      Serial1.print(m.dir == TYPE_REPLY ? '>' : '<'); Serial1.print(' ');
      Serial1.print(m.cmd); Serial1.print(' ');
      Serial1.print(m.expected); Serial1.print(' ');
      for(size_t i = 0; i < m.expected; i++)
      {
        Serial1.print(m.buffer[i], HEX); Serial1.print(' ');
      }
      Serial1.println();
    }

    void debugResponse(const MspResponse& r)
    {
      if(!debugSkip(r.cmd)) return;

      Serial1.print(r.result ? '>' : '!'); Serial1.print(' ');
      Serial1.print(r.cmd); Serial1.print(' ');
      Serial1.print(r.len); Serial1.print(' ');
      for(size_t i = 0; i < r.len; i++)
      {
        Serial1.print(r.data[i], HEX); Serial1.print(' ');
      }
      Serial1.println();
    }

    void processCommand(MspMessage& m)
    {

    }

    void processReply(MspMessage& m, Stream& s)
    {
      MspResponse r;
      r.cmd = m.cmd;
      r.result = 1;
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

        case MSP_UID:
          r.writeU32(0);
          r.writeU32(0);
          r.writeU32(0);
          break;

        case MSP_STATUS_EX:
        case MSP_STATUS:
          r.writeU16(_model.state.loopSampleInterval);
          r.writeU16(0); // i2c error count
          //         acc,     baro,    mag,     gps,     sonar,   gyro
          r.writeU16(1 << 0 | 0 << 1 | 0 << 2 | 0 << 3 | 0 << 4 | 1 << 5);
          r.writeU32(0); // flight mode flags
          r.writeU8(0); // pid profile
          r.writeU16(lrintf(_model.state.stats.getTotalLoad()));
          if (m.cmd == MSP_STATUS_EX) {
            r.writeU8(1);
            r.writeU8(0);
          } else {  // MSP_STATUS
            r.writeU16(_model.state.gyroSampleInterval); // gyro cycle time
          }

          // flight mode flags
          r.writeU8(0); // count

          // Write arming disable flags
          r.writeU8(16);  // 1 byte, flag count
          r.writeU8(0);   // 4 bytes, flags
          break;

        case MSP_NAME:
          {
            const char * name = "ESPFC";
            const int nameLen = strlen(name);
            r.writeData(name, nameLen);
          }
          break;

        case MSP_BOXNAMES:
        case MSP_BOXIDS:
          break;

        case MSP_ANALOG:
          r.writeU8(0);  // voltage
          r.writeU16(0); // mah drawn
          r.writeU16(0); // rssi
          r.writeU16(0); // amperage
          break;

        case MSP_FEATURE_CONFIG:
          r.writeU32(0);
          break;

        case MSP_BATTERY_CONFIG:
          r.writeU8(34);  // vbatmincellvoltage
          r.writeU8(43);  // vbatmaxcellvoltage
          r.writeU8(35);  // vbatwarningcellvoltage
          r.writeU16(1200); // batteryCapacity
          r.writeU8(1);  // voltageMeterSource
          r.writeU8(0);  // currentMeterSource
          break;

        case MSP_DATAFLASH_SUMMARY:
          r.writeU8(0); // FlashFS is neither ready nor supported
          r.writeU32(0);
          r.writeU32(0);
          r.writeU32(0);
          break;

        case MSP_ACC_TRIM:
          r.writeU16(0); // pitch
          r.writeU16(0); // roll
          break;

        case MSP_MIXER_CONFIG:
          r.writeU8(3); // mixerMode, QUAD_X
          r.writeU8(0); // yaw_motors_reversed
          break;

        case MSP_ATTITUDE:
          r.writeU16(lrintf(degrees(_model.state.angle.x) * 10)); // roll
          r.writeU16(lrintf(degrees(_model.state.angle.y) * 10)); // pitch
          r.writeU16(lrintf(degrees(_model.state.angle.z))); // yaw
          break;

        case MSP_BEEPER_CONFIG:
          r.writeU32(0); // beeper mask
          break;

        case MSP_BOARD_ALIGNMENT_CONFIG:
          r.writeU16(0); // roll
          r.writeU16(0); // pitch
          r.writeU16(0); // yaw
          break;

        case MSP_RX_MAP:
          for(size_t i = 0; i < 8; i++)
          {
            r.writeU8(_model.config.inputMap[i]);
          }
          break;

        case MSP_RSSI_CONFIG:
          r.writeU8(0);
          break;

        case MSP_MOTOR_CONFIG:
          r.writeU16(1050); // minthrottle
          r.writeU16(2000); // maxthrottle
          r.writeU16(1050); // mincommand
          break;

        case MSP_ARMING_CONFIG:
          r.writeU8(5);
          r.writeU8(0);
          break;

        default:
          r.result = 0;
          break;

      }
      sendResponse(r, s);
    }

    bool debugSkip(uint8_t cmd)
    {
      if(cmd == MSP_STATUS) return false;
      if(cmd == MSP_STATUS_EX) return false;
      if(cmd == MSP_BOXNAMES) return false;
      if(cmd == MSP_ANALOG) return false;
      if(cmd == MSP_ATTITUDE) return false;
      return true;
    }

    void sendResponse(MspResponse& r, Stream& s)
    {
      debugResponse(r);

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
