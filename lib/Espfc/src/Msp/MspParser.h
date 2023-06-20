#ifndef _ESPFC_MSP_PARSER_H_
#define _ESPFC_MSP_PARSER_H_

#include "Msp.h"
#include "Math/Crc.h"

namespace Espfc {

namespace Msp {

class MspParser
{
  public:
    MspParser() {}

    void parse(char c, MspMessage& msg)
    {
      switch(msg.state)
      {
        case MSP_STATE_IDLE:               // sync char 1 '$'
          if(c == '$') msg.state = MSP_STATE_HEADER_START;
          break;

        case MSP_STATE_HEADER_START:       // sync char 2 'M'
          msg.read = 0;
          msg.received = 0;
          msg.checksum = 0;
          msg.checksum2 = 0;
          if(c == 'M')
          {
            msg.version = MSP_V1;
            msg.state = MSP_STATE_HEADER_M;
          }
          else if(c == 'X')
          {
            msg.version = MSP_V2;
            msg.state = MSP_STATE_HEADER_X;
          }
          else msg.state = MSP_STATE_IDLE;
          break;

        case MSP_STATE_HEADER_M:               // type '<','>','!'
          switch(c)
          {
            case '>':
              msg.dir = MSP_TYPE_REPLY;
              msg.state = MSP_STATE_HEADER_V1;
              break;
            case '<':
              msg.dir = MSP_TYPE_CMD;
              msg.state = MSP_STATE_HEADER_V1;
              break;
            default:
              msg.state = MSP_STATE_IDLE;
          }
          break;

        case MSP_STATE_HEADER_X:                // type '<','>','!'
          switch(c)
          {
            case '>':
              msg.dir = MSP_TYPE_REPLY;
              msg.state = MSP_STATE_HEADER_V2;
              break;
            case '<':
              msg.dir = MSP_TYPE_CMD;
              msg.state = MSP_STATE_HEADER_V2;
              break;
            default:
              msg.state = MSP_STATE_IDLE;
          }
          break;

        case MSP_STATE_HEADER_V1:
          msg.buffer[msg.received++] = c;
          msg.checksum = Math::crc8_xor(msg.checksum, c);
          if(msg.received == sizeof(MspHeaderV1))
          {
            const MspHeaderV1 * hdr = reinterpret_cast<MspHeaderV1*>(msg.buffer);
            if(hdr->size > MSP_BUF_SIZE) msg.state = MSP_STATE_IDLE;
            else
            {
              msg.expected = hdr->size;
              msg.cmd = hdr->cmd;
              msg.received = 0;
              msg.state = msg.expected > 0 ? MSP_STATE_PAYLOAD_V1 : MSP_STATE_CHECKSUM_V1;
            }
          }
          break;

        case MSP_STATE_PAYLOAD_V1:
          msg.buffer[msg.received++] = c;
          msg.checksum = Math::crc8_xor(msg.checksum, c);
          if(msg.received == msg.expected)
          {
            msg.state = MSP_STATE_CHECKSUM_V1;
          }
          break;

        case MSP_STATE_CHECKSUM_V1:
          msg.state = msg.checksum == c ? MSP_STATE_RECEIVED : MSP_STATE_IDLE;
          break;

        case MSP_STATE_HEADER_V2:
          msg.buffer[msg.received++] = c;
          msg.checksum2 = Math::crc8_dvb_s2(msg.checksum2, c);
          if(msg.received == sizeof(MspHeaderV2))
          {
            const MspHeaderV2 * hdr = reinterpret_cast<MspHeaderV2*>(msg.buffer);
            if(hdr->size > MSP_BUF_SIZE) msg.state = MSP_STATE_IDLE;
            else
            {
              msg.flags = hdr->flags;
              msg.cmd = hdr->cmd;
              msg.expected = hdr->size;
              msg.received = 0;
              msg.state = msg.expected > 0 ? MSP_STATE_PAYLOAD_V2 : MSP_STATE_CHECKSUM_V2;
            }
          }
          break;

        case MSP_STATE_PAYLOAD_V2:
          msg.buffer[msg.received++] = c;
          msg.checksum2 = Math::crc8_dvb_s2(msg.checksum2, c);
          if(msg.received == msg.expected)
          {
            msg.state = MSP_STATE_CHECKSUM_V2;
          }
          break;

        case MSP_STATE_CHECKSUM_V2:
          msg.state = msg.checksum2 == c ? MSP_STATE_RECEIVED : MSP_STATE_IDLE;
          break;

        default:
          //msg.state = MSP_STATE_IDLE;
          break;
      }
    }
};

}

}

#endif
