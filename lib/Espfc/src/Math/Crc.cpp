#include "Crc.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Math {

uint8_t FAST_CODE_ATTR crc8_dvb_s2(uint8_t crc, const uint8_t a)
{
  crc ^= a;
  for (size_t i = 0; i < 8; ++i)
  {
    if (crc & 0x80)
    {
      crc = (crc << 1) ^ 0xD5;
    }
    else
    {
      crc = crc << 1;
    }
  }
  return crc;
}

uint8_t FAST_CODE_ATTR crc8_dvb_s2(uint8_t crc, const uint8_t *data, size_t len)
{
  while (len-- > 0)
  {
    crc = crc8_dvb_s2(crc, *data++);
  }
  return crc;
}

uint8_t FAST_CODE_ATTR crc8_xor(uint8_t checksum, const uint8_t a)
{
  return checksum ^ a;
}

uint8_t FAST_CODE_ATTR crc8_xor(uint8_t checksum, const uint8_t *data, int len)
{
  while (len-- > 0)
  {
    checksum = crc8_xor(checksum, *data++);
  }
  return checksum;
}

}

}