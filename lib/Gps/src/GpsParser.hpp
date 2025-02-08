#pragma once

#include "GpsProtocol.hpp"
#include <cstdint>

namespace Gps {

class UbxParser
{
public:
  void parse(uint8_t c, UbxMessage& m)
  {
    switch (m.status)
    {
      case UBX_STATE_READY:
      case UBX_STATE_IDLE:
        if (c == UBX_SYNC0) m.status = UBX_STATE_SYNC;
        else m.status = UBX_STATE_IDLE;
        break;

      case UBX_STATE_SYNC:
        if (c == UBX_SYNC1) m.status = UBX_STATE_CLASS;
        else m.status = UBX_STATE_IDLE;
        break;

      case UBX_STATE_CLASS:
        m.msgId = c;
        m.status = UBX_STATE_ID;
        break;

      case UBX_STATE_ID:
        m.msgId |= c << 8;
        m.status = UBX_STATE_LEN0;
        break;

      case UBX_STATE_LEN0:
        m.length = c;
        m.status = UBX_STATE_LEN1;
        break;

      case UBX_STATE_LEN1:
        m.length |= ((uint16_t)c << 8);
        if(m.length == 0) m.status = UBX_STATE_CRC0;
        else if(m.length >= 511) m.status = UBX_STATE_IDLE;
        else m.status = UBX_STATE_PAYLOAD;
        break;

      case UBX_STATE_PAYLOAD:
        if(m.written < 511) {
          m.payload[m.written] = c;
          m.written++;
        }
        if(m.written == m.length) m.status = UBX_STATE_CRC0;
        break;

      case UBX_STATE_CRC0:
        m.crc = c;
        m.status = UBX_STATE_CRC1;
        break;

      case UBX_STATE_CRC1:
        m.crc |= c << 8;
        if(m.crc == m.checksum()) m.status = UBX_STATE_READY;
        else m.status = UBX_STATE_IDLE;
        break;
    }
  }
};

class NmeaParser
{
public:
  void parse(uint8_t c, NmeaMessage& m)
  {
    switch(m.status)
    {
      case NMEA_STATE_READY:
      case NMEA_STATE_IDLE:
        if(c == '$') m.status = NMEA_STATE_PAYLOAD;
        break;

      case NMEA_STATE_PAYLOAD:
        if (c == '\r') m.status = NMEA_STATE_END;
        else if (c == '\n') m.status = NMEA_STATE_IDLE;
        else {
          if (m.length < 511) {
            m.payload[m.length++] = c;
            m.payload[m.length] = '\0';
          }
        }
        break;

      case NMEA_STATE_END:
        if (c == '\n') {
          m.status = NMEA_STATE_READY;
        }
        break;
    }
  }
};

}
