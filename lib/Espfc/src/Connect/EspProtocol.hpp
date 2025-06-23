#pragma once

#include <cstdint>

namespace Espfc::Connect {

enum EspCommand: uint8_t {
  ESP_CMD_VERSION = 0,
  ESP_CMD_STATUS = 1,
  ESP_CMD_STATISTICS = 2,
};

static const uint8_t ESP_API_VERSION_MAJOR = 0x01;
static const uint8_t ESP_API_VERSION_MINOR = 0x00;

enum EspCmdHardwareType: uint8_t {
  ESP_HW_TYPE_UNKNOWN = 0x00,
  ESP_HW_TYPE_ESP32   = 0x01,
  ESP_HW_TYPE_ESP32S2 = 0x02,
  ESP_HW_TYPE_ESP32S3 = 0x03,
  ESP_HW_TYPE_ESP32C3 = 0x04,
  ESP_HW_TYPE_ESP8266 = 0x0f,
  ESP_HW_TYPE_RP2040  = 0x10,
  ESP_HW_TYPE_RP2350  = 0x11,
};

struct EspCmdVersion {
  uint8_t apiMajor;
  uint8_t apiMinor;
  uint8_t hwType;
  uint16_t capabilities;
  uint8_t fwVersion[16];
  uint8_t fwRevision[16];
} __attribute__((packed));

struct EspCmdStatus {
  uint16_t sensors;
  uint16_t gyroTimeUs;
  uint32_t modeSwitchMask;
  uint32_t modeActiveMask;
  uint32_t armingDisableFlags;
} __attribute__((packed));

struct EspCmdStatistics {
  uint32_t uptimeMs;
  uint8_t cpuLoad;
  uint8_t cpu0Load;
  uint8_t cpu1Load;
  uint32_t freeHeap;
  uint32_t totalHeap;
  uint32_t flashTotal;
  uint32_t flashUsed;
} __attribute__((packed));

}
