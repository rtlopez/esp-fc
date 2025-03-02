#pragma once

#include <cstddef>
#include <cstdint>

enum EscProtocol {
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
  ESC_PROTOCOL_COUNT
};

struct EscConfig
{
  int timer;
  EscProtocol protocol;
  int rate;
  bool async;
  bool dshotTelemetry;
};

#define PWM_TO_DSHOT(v) (((v - 1000) * 2) + 47)
#define ESC_PROTOCOL_SANITIZE(p) (p > ESC_PROTOCOL_DSHOT600 && p != ESC_PROTOCOL_DISABLED ? ESC_PROTOCOL_DSHOT600 : p)

class EscDriverBase
{
  public:
    static constexpr size_t DSHOT_BIT_COUNT = 16;
    static constexpr uint32_t INVALID_TELEMETRY_VALUE = 0xffff;
    static constexpr int SECONDS_PER_MINUTE = 60;
    static constexpr float ERPM_PER_LSB = 100.0f;

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

    static const char * const * getProtocolNames();
    static const char * const getProtocolName(EscProtocol protocol);

#if defined(UNIT_TEST)
    int begin(const EscConfig& conf) { return 1; }
    void end() {}
    int attach(size_t channel, int pin, int pulse) { return 1; }
    int write(size_t channel, int pulse) { return 1; }
    void apply() {}
    int pin(size_t channel) const { return -1; }
    uint32_t telemetry(size_t channel) const { return 0; }
#endif
};
