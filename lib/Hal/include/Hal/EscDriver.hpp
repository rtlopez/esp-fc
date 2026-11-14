#pragma once

#include <cstddef>
#include <cstdint>

namespace Hal {

enum EscProtocol
{
  ESC_PROTOCOL_PWM,
  ESC_PROTOCOL_ONESHOT125,
  ESC_PROTOCOL_ONESHOT42,
  ESC_PROTOCOL_MULTISHOT,
  ESC_PROTOCOL_BRUSHED,
  ESC_PROTOCOL_DSHOT150,
  ESC_PROTOCOL_DSHOT300,
  ESC_PROTOCOL_DSHOT600,
  ESC_PROTOCOL_PROSHOT,
  ESC_PROTOCOL_DISABLED,
  ESC_PROTOCOL_COUNT,
};

// Selects platform resources (timer, peripheral instance) used by the driver.
enum EscRole : uint8_t
{
  ESC_ROLE_MOTOR,
  ESC_ROLE_SERVO,
  ESC_ROLE_COUNT,
};

struct EscConfig
{
  EscRole role;
  EscProtocol protocol;
  int rate;
  bool async;
  bool dshotTelemetry;
};

namespace Detail {
class EscDriverImpl;
}

class EscDriver
{
public:
#if defined(ESP32) && !defined(ESP32S2) && !defined(ESP32S3) && !defined(ESP32C3)
  static constexpr size_t CHANNEL_COUNT = 8;
#elif defined(ARCH_RP2040)
  static constexpr size_t CHANNEL_COUNT = 8;
#else
  static constexpr size_t CHANNEL_COUNT = 4;
#endif

  static constexpr size_t DSHOT_BIT_COUNT = 16;
  static constexpr uint32_t INVALID_TELEMETRY_VALUE = 0xffff;
  static constexpr int SECONDS_PER_MINUTE = 60;
  static constexpr float ERPM_PER_LSB = 100.0f;

  bool begin(const EscConfig& conf);
  void end();
  bool attach(size_t channel, int pin, int pulse);
  bool write(size_t channel, int pulse);
  void apply();
  int pin(size_t channel) const;
  uint32_t telemetry(size_t channel) const;

  static constexpr EscProtocol sanitizeProtocol(int protocol)
  {
    if (protocol < 0) return ESC_PROTOCOL_DISABLED;
    if (protocol > ESC_PROTOCOL_DSHOT600 && protocol != ESC_PROTOCOL_DISABLED) return ESC_PROTOCOL_DSHOT600;
    return static_cast<EscProtocol>(protocol);
  }

  static constexpr int pwmToDshot(int pulse)
  {
    return ((pulse - 1000) * 2) + 47;
  }

  static uint16_t dshotConvert(uint16_t pulse);
  static uint16_t dshotEncode(uint16_t value, bool inverted = false);

  /**
   * @param data expected data layout (bits): duration0(15), level0(1), duration(15), level1(1)
   * @param len number of data items
   * @param bitLen duration of single bit
   * @return uint32_t raw gcr value
   */
  static uint32_t extractTelemetryGcr(uint32_t* data, size_t len, uint32_t bitLen);
  static uint32_t durationToBitLen(uint32_t duration, uint32_t len);
  static uint32_t pushBits(uint32_t value, uint32_t bitVal, size_t bitLen);
  static uint32_t gcrToRawValue(uint32_t value);

  static float getErpmToHzRatio(int poles);
  static uint32_t convertToErpm(uint32_t value);
  static uint32_t convertToValue(uint32_t value);

  static const char* const* getProtocolNames();
  static const char* getProtocolName(EscProtocol protocol);

private:
  Detail::EscDriverImpl* _impl = nullptr;
};

} // namespace Hal
