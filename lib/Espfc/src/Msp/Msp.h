#ifndef _ESPFC_MSP_MSP_H_
#define _ESPFC_MSP_MSP_H_

#include <cstdint>
#include "Hal/Pgm.h"
#include "Math/Crc.h"

extern "C" {
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_common.h"
#include "msp/msp_protocol_v2_betaflight.h"
}

namespace Espfc {

namespace Msp {

static const size_t MSP_BUF_SIZE = 192;
static const size_t MSP_BUF_OUT_SIZE = 240;

enum MspState {
  MSP_STATE_IDLE,
  MSP_STATE_HEADER_START,

  MSP_STATE_HEADER_M,
  MSP_STATE_HEADER_V1,
  MSP_STATE_PAYLOAD_V1,
  MSP_STATE_CHECKSUM_V1,

  MSP_STATE_HEADER_X,
  MSP_STATE_HEADER_V2,
  MSP_STATE_PAYLOAD_V2,
  MSP_STATE_CHECKSUM_V2,

  MSP_STATE_RECEIVED,
};

enum MspType {
  MSP_TYPE_CMD,
  MSP_TYPE_REPLY
};

enum MspVersion {
  MSP_V1,
  MSP_V2
};

struct MspHeaderV1 {
    uint8_t size;
    uint8_t cmd;
} __attribute__((packed));

struct MspHeaderV2 {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} __attribute__((packed));

class MspMessage
{
  public:
    MspMessage(): state(MSP_STATE_IDLE), expected(0), received(0), read(0) {}
    MspState state;
    MspType dir;
    MspVersion version;
    uint8_t flags;
    uint16_t cmd;
    uint16_t expected;
    uint16_t received;
    uint16_t read;
    uint8_t checksum;
    uint8_t checksum2;
    uint8_t buffer[MSP_BUF_SIZE];

    bool isReady() const
    {
      return state == MSP_STATE_RECEIVED;
    }

    bool isCmd() const
    {
      return dir == MSP_TYPE_CMD;
    }

    bool isIdle() const
    {
      return state == MSP_STATE_IDLE;
    }

    int remain() const
    {
      return received - read;
    }

    void advance(size_t size)
    {
      read += size;
    }

    uint8_t readU8()
    {
      return buffer[read++];
    }

    uint16_t readU16()
    {
      uint16_t ret;
      ret = readU8();
      ret |= readU8() << 8;
      return ret;
    }

    uint32_t readU32()
    {
      uint32_t ret;
      ret = readU8();
      ret |= readU8() <<  8;
      ret |= readU8() << 16;
      ret |= readU8() << 24;
      return ret;
    }
};

class MspResponse
{
  public:
    MspResponse(): len(0) {}
    MspVersion version;
    uint16_t cmd;
    int8_t  result;
    uint8_t direction;
    uint16_t len;
    uint8_t data[MSP_BUF_OUT_SIZE];

    int remain() const
    {
      return MSP_BUF_OUT_SIZE - len;
    }

    void advance(size_t size)
    {
      len += size;
    }

    void writeData(const char * v, int size)
    {
      while(size-- > 0) writeU8(*v++);
    }

    void writeString(const char * v)
    {
      while(*v) writeU8(*v++);
    }

    void writeString(const __FlashStringHelper *ifsh)
    {
      PGM_P p = reinterpret_cast<PGM_P>(ifsh);
      while(true)
      {
        uint8_t c = pgm_read_byte(p++);
        if (c == 0) break;
        writeU8(c);
      }
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

    size_t serialize(uint8_t * buff, size_t len_max) const
    {
      switch(version)
      {
        case MSP_V1:
          return serializeV1(buff, len_max);
        case MSP_V2:
          return serializeV2(buff, len_max);
      }
      return 0;
    }

    size_t serializeV1(uint8_t * buff, size_t len_max) const
    {
      // not enough space in target buffer
      if(len + 6 > len_max) return 0;

      buff[0] = '$';
      buff[1] = 'M';
      buff[2] = result == -1 ? '!' : '>';
      buff[3] = len;
      buff[4] = cmd;

      uint8_t checksum = Math::crc8_xor(0, &buff[3], 2);
      size_t i = 5;
      for(size_t j = 0; j < len; j++)
      {
        checksum = Math::crc8_xor(checksum, data[j]);
        buff[i++] = data[j];
      }
      buff[i] = checksum;

      return i + 1;
    }

    size_t serializeV2(uint8_t * buff, size_t len_max) const
    {
      // not enough space in target buffer
      if(len + 9 > len_max) return 0;

      buff[0] = '$';
      buff[1] = 'X';
      buff[2] = result == -1 ? '!' : '>';
      buff[3] = 0;
      buff[4] = cmd & 0xff;
      buff[5] = (cmd >> 8) & 0xff;
      buff[6] = len & 0xff;
      buff[7] = (len >> 8) & 0xff;

      uint8_t checksum = Math::crc8_dvb_s2(0, &buff[3], 5);
      size_t i = 8;
      for(size_t j = 0; j < len; j++)
      {
        checksum = Math::crc8_dvb_s2(checksum, data[j]);
        buff[i++] = data[j];
      }
      buff[i] = checksum;

      return i + 1;
    }
};

}

}

#endif
