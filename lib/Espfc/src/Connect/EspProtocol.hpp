#pragma once

#include <cstdint>

namespace Espfc::Connect
{

enum EspCommand : uint8_t
{
  ESP_CMD_VERSION = 0x01,
  ESP_CMD_STATUS = 0x02,
  ESP_CMD_STATISTICS = 0x03,
  ESP_CMD_ATTITUDE = 0x04,
  ESP_CMD_SENSORS = 0x05,
  ESP_CMD_INPUT = 0x06,
  ESP_CMD_OUTPUT = 0x07,
  ESP_CMD_VOLTAGE = 0x08,
  ESP_CMD_CURRENT = 0x09,
  ESP_CMD_GPS = 0x0a,
  ESP_CMD_GPS_INFO = 0x0b,
  ESP_CMD_RPM_TLM = 0x0c,
  ESP_CMD_DEBUG = 0x0f,

  ESP_CMD_MODE_NAMES = 0x10,
  ESP_CMD_FEATURE_NAMES = 0x11,
  ESP_CMD_DEBUG_NAMES = 0x12,
  ESP_CMD_SERIAL_NAMES = 0x13,
  ESP_CMD_PID_NAMES = 0x14,

  ESP_CMD_INPUT_CONFIG = 0x20,
  ESP_CMD_INPUT_CHANNEL_CONFIG = 0x21,
  ESP_CMD_OUTPUT_CONFIG = 0x22,
  ESP_CMD_OUTPUT_CHANNEL_CONFIG = 0x23,
  ESP_CMD_GYRO_CONFIG = 0x24,
  ESP_CMD_ACCEL_CONFIG = 0x25,
  ESP_CMD_SERIAL_CONFIG = 0x26,
  ESP_CMD_VOLTAGE_CONFIG = 0x27,
  ESP_CMD_CURRENT_CONFIG = 0x28,
  ESP_CMD_PID_CONFIG = 0x29,
  ESP_CMD_PID_COMMON_CONFIG = 0x2a,
  ESP_CMD_MODES_CONFIG = 0x2b,
  ESP_CMD_FAILSAFE_CONFIG = 0x2c,
  ESP_CMD_BLACKBOX_CONFIG = 0x2d,
  ESP_CMD_MIXER_CONFIG = 0x2e,
  ESP_CMD_RPM_FILTER_CONFIG = 0x2f,
  ESP_CMD_DYN_NOTCH_CONFIG = 0x30,
  ESP_CMD_VTX_CONFIG = 0x31,
  ESP_CMD_GPS_CONFIG = 0x32,
  ESP_CMD_BARO_CONFIG = 0x33,
  ESP_CMD_MAG_CONFIG = 0x34,
  ESP_CMD_FEATURE_CONFIG = 0x35,
  ESP_CMD_MODEL_CONFIG = 0x36,
  ESP_CMD_CALIBRATE = 0x37,
  ESP_CMD_ESC_PASSTHROUGH = 0x38,
  ESP_CMD_ALIGNMENT_CONFIG = 0x39,

  ESP_CMD_FLASH_STATUS = 0x40,
  ESP_CMD_FLASH_READ = 0x41,
  ESP_CMD_FLASH_ERASE = 0x42,

  ESP_CMD_DISABLE_ARM = 0xf0,
  ESP_CMD_DEFAULTS = 0xf1,
  ESP_CMD_SAVE = 0xf2,
  ESP_CMD_REBOOT = 0xf3,
};

static const uint8_t ESP_API_VERSION_MAJOR = 0x01;
static const uint8_t ESP_API_VERSION_MINOR = 0x00;

enum EspCmdHardwareType : uint8_t
{
  ESP_HW_TYPE_UNKNOWN = 0x00,
  ESP_HW_TYPE_ESP32 = 0x01,
  ESP_HW_TYPE_ESP32S2 = 0x02,
  ESP_HW_TYPE_ESP32S3 = 0x03,
  ESP_HW_TYPE_ESP32C3 = 0x04,
  ESP_HW_TYPE_ESP8266 = 0x0f,
  ESP_HW_TYPE_RP2040 = 0x10,
  ESP_HW_TYPE_RP2350 = 0x11,
};

struct EspCmdVersion
{
  uint8_t apiMajor;
  uint8_t apiMinor;
  uint8_t hwType;
  uint32_t capabilities;
  uint8_t fwVersion[16];
  uint8_t fwRevision[16];
} __attribute__((packed));

struct EspCmdStatus
{
  uint16_t sensors;
  uint16_t gyroTimeUs;
  uint32_t modeSwitchMask;
  uint32_t modeActiveMask;
  uint32_t armingDisableFlags;
} __attribute__((packed));

struct EspCmdStatistics
{
  uint32_t uptimeMs;
  uint8_t cpuLoad;
  uint8_t cpu0Load;
  uint8_t cpu1Load;
  uint32_t heapTotal;
  uint32_t heapFree;
  uint32_t flashTotal;
  uint32_t flashUsed;
} __attribute__((packed));

struct EspCmdAttitude
{
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t w;
} __attribute__((packed));

struct EspCmdSensors
{
  int16_t gyro[3];
  int16_t accel[3];
  int16_t mag[3];
  int16_t baroAlt;
} __attribute__((packed));

struct EspCmdInput
{
  uint8_t channelCount; // number of input channels
  uint16_t channels[16];  // up to 16 channels
} __attribute__((packed));

struct EspCmdOutput
{
  uint8_t channelCount; // number of output channels
  uint16_t channels[8];  // up to 8 channels
} __attribute__((packed));

struct EspCmdVoltage
{
  uint16_t voltage; // in 0.01V units
  uint8_t cellCount; // number of cells in the battery
} __attribute__((packed));

struct EspCmdCurrent
{
  uint16_t current; // in 0.01A units
  uint32_t consumption; // in mAh
} __attribute__((packed));

struct EspCmdGps
{
  uint32_t time; // ms since epoch
  uint8_t fixType; // 0: no fix, 1: 2D fix, 2: 3D fix, 3: DGPS fix, 4: RTK fix
  uint8_t sats; // number of satellites used in the fix
  int32_t latitude; // in degrees * 1e7
  int32_t longitude; // in degrees * 1e7
  int32_t altitude; // in meters * 1000
  int16_t speed; // in cm/s
  int16_t course; // in degrees * 100
} __attribute__((packed));

struct EspCmdGpsInfo
{
  uint8_t numSats;
  struct SvInfo {
    uint8_t gnssId;
    uint8_t id;
    uint8_t quality;
    uint8_t cno;
  } svs[32]; // up to 32 SVs
} __attribute__((packed));

struct EspCmdRpmtlm
{
  uint8_t channelCount;
  struct Channel {
    uint32_t rpm; // in RPM
    uint16_t invalidPct; // percentage of invalid samples
    uint8_t temperature; // in degrees Celsius
    uint16_t voltage; // in 0.01V units
    uint16_t current; // in 0.01A units
  } channels[4]; // up to 4 channels
} __attribute__((packed));

struct EspCmdDebug
{
  uint16_t debug[8];
} __attribute__((packed));

struct EspCmdNames
{
  char names[1];
} __attribute__((packed));

struct EspCmdInputChannelConfig
{
  uint8_t map; // channel mapping
  uint16_t min; // minimum range
  uint16_t max; // maximum range
  uint8_t fsMode; // failsafe mode
  uint16_t fsValue; // failsafe value
} __attribute__((packed));

struct EspCmdInputChannelConfigRequest
{
  uint8_t channel;
  EspCmdInputChannelConfig config;
} __attribute__((packed));

struct EspCmdInputChannelConfigResponse
{
  uint8_t count;
  EspCmdInputChannelConfig configs[INPUT_CHANNELS]; // up to 16 channels
} __attribute__((packed));

struct EspCmdOutputConfig
{
  uint8_t protocol;
  uint8_t async;
  uint16_t rate;
  uint16_t servoRate;
  uint16_t minCommand;
  uint16_t minThrottle;
  uint16_t maxThrottle;
  uint16_t digitalIdle;
  uint8_t digitalTlm;
  uint8_t motorPoles;
  uint8_t motorLimit;
  uint8_t throttleLimitType;
  uint8_t throttleLimitPercent;
} __attribute__((packed));


struct EspCmdOutputChannelConfig
{
  uint16_t min; // minimum value
  uint16_t neutral; // neutral value
  uint16_t max; // maximum value
  uint8_t servo; // servo or motor
  uint8_t reverse; // normal or reversed
  int8_t pin;
} __attribute__((packed));

struct EspCmdOutputChannelConfigRequest
{
  uint8_t channel;
  EspCmdOutputChannelConfig config;
} __attribute__((packed));

struct EspCmdOutputChannelConfigResponse
{
  uint8_t count;
  EspCmdOutputChannelConfig configs[OUTPUT_CHANNELS]; // up to 8 channels
} __attribute__((packed));

struct EspCmdLpfConfig {
  uint8_t type;
  uint16_t freq;
} __attribute__((packed));

struct EspCmdDynLpfConfig {
  uint8_t type;
  uint16_t minFreq;
  uint16_t maxFreq;
} __attribute__((packed));

struct EspCmdNotchConfig {
  uint8_t type;
  uint16_t freq;
  uint16_t bandwidth;
} __attribute__((packed));

struct EspCmdDynNotchConfig {
  uint8_t count;
  uint16_t q;
  uint16_t minFreq;
  uint16_t maxFreq;
} __attribute__((packed));

struct EspCmdRpmNotchConfig {
  uint8_t harmonics;
  uint16_t q;
  uint16_t minFreq;
} __attribute__((packed));

struct EspCmdGyroConfig
{
  uint8_t align;
  EspCmdLpfConfig lpf[3];
  EspCmdDynLpfConfig dynLpf;
  EspCmdNotchConfig notch[2];
  EspCmdDynNotchConfig dynNotch;
  EspCmdRpmNotchConfig rpmNotch;
} __attribute__((packed));

struct EspCmdAccelConfig
{
  uint8_t align;
  EspCmdLpfConfig lpf;
} __attribute__((packed));

struct EspCmdSerialConfig
{
  uint8_t baud;
  uint8_t func;
} __attribute__((packed));

struct EspCmdSerialConfigRequest
{
  uint8_t serial;
  EspCmdSerialConfig config;
} __attribute__((packed));

struct EspCmdSerialConfigResponse
{
  uint8_t serialCount;
  EspCmdSerialConfig serial[6];
} __attribute__((packed));

struct EspCmdVoltageConfig
{
  uint8_t source;
  uint16_t scale; // scale factor
  uint16_t offset; // offset value
  uint8_t resDiv; // resistor divider value
  uint8_t resMult; // resistor multiplier value
  uint16_t cellWarning; // cell warning threshold
  uint16_t cellCritical; // cell critical threshold
} __attribute__((packed));
struct EspCmdCurrentConfig
{
  uint8_t source;
  uint16_t scale; // scale factor
  int16_t offset; // offset value
} __attribute__((packed));

struct EspCmdPidConfig
{
  uint8_t p; // P value
  uint8_t i; // I value
  uint8_t d; // D value
  uint8_t f; // F value
} __attribute__((packed));

struct EspCmdPidConfigRequest
{
  uint8_t pid;
  EspCmdPidConfig config;
} __attribute__((packed));

struct EspCmdPidConfigResponse
{
  uint8_t pidCount;
  EspCmdPidConfig config[8];
} __attribute__((packed));

struct EspCmdPidCommonConfig
{
  uint8_t loopSync; // loop sync mode
  uint8_t tpaScale; // TPA scale
  uint16_t tpaBreakpoint; // TPA breakpoint
  uint8_t levelAngleLimit; // level angle limit
  uint16_t levelRateLimit; // level rate limit
  EspCmdLpfConfig levelPtermFilter; // level P-term filter
  EspCmdLpfConfig yawFilter; // yaw filter
  EspCmdLpfConfig dtermFilter[2]; // D-term filter
  EspCmdNotchConfig dtermNotchFilter; // D-term notch filter
  uint8_t itermRelax; // I-term relax mode
  uint8_t itermRelaxType; // I-term relax type
  uint8_t itermRelaxCutoff; // I-term relax cutoff
  uint16_t itermLimit; // I-term limit
} __attribute__((packed));

struct EspCmdModesConfig
{
  uint8_t modeCount;
  struct Mode {
    uint8_t id; // mode ID
    uint8_t ch; // channel
    uint16_t min; // rate limit
    uint16_t max; // angle limit
  } modes[8]; // up to 8 modes
} __attribute__((packed));

struct EspCmdFailsafeConfig
{
  uint8_t delay; // failsafe delay
} __attribute__((packed));

struct EspCmdBlackboxConfig
{
  uint8_t device; // blackbox device
  uint8_t denom; // blackbox rate denominator
  uint8_t mode; // blackbox mode
  uint32_t fieldMask; // blackbox field mask
  uint8_t debugMode; // debug mode
  uint8_t debugAxis; // debug axis
} __attribute__((packed));

struct EspCmdMixerConfig
{
  uint8_t type; // mixer type
  uint8_t yawReverse; // yaw reverse
  uint8_t sync; // mixer sync
} __attribute__((packed));

struct EspCmdScalerConfig
{
  uint8_t dimension; // scaler dimension
  uint8_t channel; // scaler channel
  int16_t minScale; // minimum scale
  int16_t maxScale; // maximum scale
} __attribute__((packed));

struct EspCmdVtxConfig
{
  uint8_t power; // VTX power
  uint8_t band; // VTX band
  uint8_t channel; // VTX channel
  uint8_t pitMode; // pit mode
} __attribute__((packed));

struct EspCmdGpsConfig
{
  uint8_t minSats; // minimum satellites
  uint8_t setHomeOnce; // set home position once
} __attribute__((packed));

struct EspCmdBaroConfig
{
  EspCmdLpfConfig lpf; // barometer low-pass filter
} __attribute__((packed));

struct EspCmdMagConfig
{
  uint8_t align; // magnetometer alignment
  EspCmdLpfConfig lpf; // magnetometer low-pass filter
} __attribute__((packed));

struct EspCmdFeatureConfig
{
  uint32_t features; // feature flags
} __attribute__((packed));

struct EspCmdModelConfig
{
  uint8_t name[32]; // model name
} __attribute__((packed));

struct EspCmdAlignmentConfig
{
  uint8_t align[3]; // gyro alignment
} __attribute__((packed));

struct EspCmdFlashStatus
{
  uint32_t totalSize; // total flash size
  uint32_t usedSize; // used flash size
  uint32_t freeSize; // free flash size
} __attribute__((packed));

struct EspCmdFlashReadRequest
{
  uint32_t address; // flash address to read from
  uint32_t size; // size of data to read
} __attribute__((packed));

struct EspCmdFlashReadResponse
{
  uint32_t address; // flash address read from
  uint32_t size; // size of data read
  uint8_t data[1]; // data read from flash (variable length)
} __attribute__((packed));

enum EspCmdInputType: uint8_t
{
  RX_NONE = 0,
  RX_SERIAL_IBUS = 1,
  RX_SERIAL_SBUS,
  RX_SERIAL_CRSF,
  RX_ESPNOW = 0x10,
  RX_PPM = 0x11,
};

struct EspCmdInputConfig
{
  uint8_t type;
  uint8_t deadband;
  uint8_t smoothing;
  int16_t mid;
  int16_t min;
  int16_t max;
} __attribute__((packed));

}
