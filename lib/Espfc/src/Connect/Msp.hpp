#pragma once

#include <cstdint>
#include <cstddef>
#include "Hal/Pgm.h"

namespace Espfc {

namespace Connect {

constexpr size_t MSP_BUF_SIZE = 192;
constexpr size_t MSP_BUF_OUT_SIZE = 240;

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
  MspMessage();
  bool isReady() const;
  bool isCmd() const;
  bool isIdle() const;
  int remain() const;
  void advance(size_t size);
  uint8_t readU8();
  uint16_t readU16();
  uint32_t readU32();

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
};

class MspResponse
{
public:
  MspResponse();

  MspVersion version;
  uint16_t cmd;
  int8_t  result;
  uint8_t direction;
  uint16_t len;
  uint8_t data[MSP_BUF_OUT_SIZE];

  int remain() const;
  void advance(size_t size);
  void writeData(const char * v, int size);
  void writeString(const char * v);
  void writeString(const __FlashStringHelper *ifsh);
  void writeU8(uint8_t v);
  void writeU16(uint16_t v);
  void writeU32(uint32_t v);
  size_t serialize(uint8_t * buff, size_t len_max) const;
  size_t serializeV1(uint8_t * buff, size_t len_max) const;
  size_t serializeV2(uint8_t * buff, size_t len_max) const;
};

}

}
