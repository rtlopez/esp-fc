#include "Hal/EscDriver.hpp"
#include "Hal/FastCode.hpp"

#if defined(ESP32C3)
#include "ESP32/EscDriverEsp32c3.hpp"
#elif defined(ESP32)
#include "ESP32/EscDriverEsp32.hpp"
#elif defined(ESP8266)
#include "ESP8266/EscDriverEsp8266.hpp"
#elif defined(ARCH_RP2040)
#include "RP2040/EscDriverRP2040.hpp"
#elif defined(UNIT_TEST)
#include "Native/EscDriverNative.hpp"
#else
#error "Unsupported platform"
#endif

namespace Hal {

namespace {

Detail::EscDriverImpl escDriverImpls[ESC_ROLE_COUNT];

} // namespace

bool EscDriver::begin(const EscConfig& conf)
{
  if (conf.role >= ESC_ROLE_COUNT) return false;
  _impl = &escDriverImpls[conf.role];
  return _impl->begin(conf);
}

void EscDriver::end()
{
  if (_impl) _impl->end();
}

bool EscDriver::attach(size_t channel, int pin, int pulse)
{
  if (!_impl || channel >= CHANNEL_COUNT || pin < 0) return false;
  return _impl->attach(channel, pin, pulse);
}

bool ISR_CODE_ATTR EscDriver::write(size_t channel, int pulse)
{
  if (!_impl || channel >= CHANNEL_COUNT) return false;
  _impl->write(channel, pulse);
  return true;
}

void ISR_CODE_ATTR EscDriver::apply()
{
  if (_impl) _impl->apply();
}

int EscDriver::pin(size_t channel) const
{
  if (!_impl || channel >= CHANNEL_COUNT) return -1;
  return _impl->pin(channel);
}

uint32_t ISR_CODE_ATTR EscDriver::telemetry(size_t channel) const
{
  if (!_impl || channel >= CHANNEL_COUNT) return 0;
  return _impl->telemetry(channel);
}

const char* const* EscDriver::getProtocolNames()
{
  static const char* const protocols[] = {"PWM",      "ONESHOT125", "ONESHOT42",   "MULTISHOT", "BRUSHED", "DSHOT150",
                                          "DSHOT300", "DSHOT600",   "PROSHOT1000", "DISABLED",  nullptr};
  return protocols;
}

const char* EscDriver::getProtocolName(EscProtocol protocol)
{
  if (protocol < 0 || protocol >= ESC_PROTOCOL_COUNT) return "?";
  return getProtocolNames()[protocol];
}

uint16_t ISR_CODE_ATTR EscDriver::dshotConvert(uint16_t pulse)
{
  return pulse > 1000 ? pwmToDshot(pulse) : 0;
}

uint16_t ISR_CODE_ATTR EscDriver::dshotEncode(uint16_t value, bool inverted)
{
  value <<= 1;

  // compute checksum
  int csum = 0;
  int csum_data = value;
  for (int i = 0; i < 3; i++)
  {
    csum ^= csum_data; // xor
    csum_data >>= 4;
  }
  if (inverted)
  {
    csum = ~csum;
  }
  csum &= 0xf;

  return (value << 4) | csum;
}

uint32_t FAST_CODE_ATTR EscDriver::durationToBitLen(uint32_t duration, uint32_t len)
{
  return (duration + (len >> 1)) / len;
}

uint32_t FAST_CODE_ATTR EscDriver::pushBits(uint32_t value, uint32_t bitVal, size_t bitLen)
{
  while (bitLen--)
  {
    value <<= 1;
    value |= bitVal;
  }
  return value;
}

/**
 * @param data expected data layout (bits): duration0(15), level0(1), duration(15), level1(1)
 * @param len number of data items
 * @param bitLen duration of single bit
 * @return uint32_t raw gcr value
 */
uint32_t FAST_CODE_ATTR EscDriver::extractTelemetryGcr(uint32_t* data, size_t len, uint32_t bitLen)
{
  int bitCount = 0;
  uint32_t value = 0;
  for (size_t i = 0; i < len; i++)
  {
    uint32_t duration0 = data[i] & 0x7fff;
    if (!duration0) break;
    uint32_t level0 = (data[i] >> 15) & 0x01;
    uint32_t len0 = durationToBitLen(duration0, bitLen);
    if (len0)
    {
      value = pushBits(value, level0, len0);
      bitCount += len0;
    }

    uint32_t duration1 = (data[i] >> 16) & 0x7fff;
    if (!duration1) break;
    uint32_t level1 = (data[i] >> 31) & 0x01;
    uint32_t len1 = durationToBitLen(duration1, bitLen);
    if (len1)
    {
      value = pushBits(value, level1, len1);
      bitCount += len1;
    }
  }

  // fill missing bits with 1
  if (bitCount < 21)
  {
    value = pushBits(value, 0x1, 21 - bitCount);
  }

  return value;
}

float FAST_CODE_ATTR EscDriver::getErpmToHzRatio(int poles)
{
  return ERPM_PER_LSB / SECONDS_PER_MINUTE / (poles / 2.0f);
}

uint32_t FAST_CODE_ATTR EscDriver::convertToErpm(uint32_t value)
{
  if (!value) return 0;

  if (!value || value == INVALID_TELEMETRY_VALUE)
  {
    return INVALID_TELEMETRY_VALUE;
  }

  // Convert period to erpm * 100
  return (1000000 * 60 / 100 + value / 2) / value;
}

uint32_t FAST_CODE_ATTR EscDriver::convertToValue(uint32_t value)
{
  // eRPM range
  if (value == 0x0fff)
  {
    return 0;
  }

  // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
  return (value & 0x01ff) << ((value & 0xfe00) >> 9);
}

uint32_t FAST_CODE_ATTR EscDriver::gcrToRawValue(uint32_t value)
{
  value = value ^ (value >> 1); // extract gcr

  constexpr uint32_t iv = 0xffffffff; // invalid code
  // First bit is start bit so discard it.
  value &= 0xfffff;
  static const uint32_t decode[32] = {
      iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
      iv, iv, 2,  3,  iv, 5,  6,  7,  iv, 0, 8,  1,  iv, 4,  12, iv,
  };

  uint32_t decodedValue = decode[value & 0x1f];
  decodedValue |= decode[(value >> 5) & 0x1f] << 4;
  decodedValue |= decode[(value >> 10) & 0x1f] << 8;
  decodedValue |= decode[(value >> 15) & 0x1f] << 12;

  uint32_t csum = decodedValue;
  csum = csum ^ (csum >> 8); // xor bytes
  csum = csum ^ (csum >> 4); // xor nibbles

  if ((csum & 0xf) != 0xf || decodedValue > 0xffff)
  {
    return INVALID_TELEMETRY_VALUE;
  }
  value = decodedValue >> 4;

  return value;
}

} // namespace Hal
