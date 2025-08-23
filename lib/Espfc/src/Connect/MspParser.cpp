#include "Connect/MspParser.hpp"
#include "Utils/Crc.hpp"

namespace Espfc {

namespace Connect {

MspParser::MspParser() {}

void MspParser::parse(char c, MspMessage& msg)
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
      switch(c)
      {
        case 'M':
          msg.version = MSP_V1;
          msg.variant = MSP_BF;
          msg.state = MSP_STATE_HEADER_M;
          break;
        case 'E':
          msg.version = MSP_V1;
          msg.variant = MSP_ESP;
          msg.state = MSP_STATE_HEADER_M;
          break;
        case 'X':
          msg.version = MSP_V2;
          msg.variant = MSP_BF;
          msg.state = MSP_STATE_HEADER_X;
          break;
        default:
          msg.state = MSP_STATE_IDLE;
      }
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
      msg.checksum = Utils::crc8_xor(msg.checksum, c);
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
      msg.checksum = Utils::crc8_xor(msg.checksum, c);
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
      msg.checksum2 = Utils::crc8_dvb_s2(msg.checksum2, c);
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
      msg.checksum2 = Utils::crc8_dvb_s2(msg.checksum2, c);
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

}

}
