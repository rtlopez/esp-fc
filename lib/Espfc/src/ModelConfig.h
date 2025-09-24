#ifndef _ESPFC_MODEL_CONFIG_H_
#define _ESPFC_MODEL_CONFIG_H_

#include "Target/Target.h"
#include "EscDriver.h"
#include "Utils/Filter.h"
#include "Device/BusDevice.h"
#include "Device/GyroDevice.h"
#include "Device/MagDevice.h"
#include "Device/BaroDevice.h"
#include "Device/SerialDevice.h"
#include "Device/InputPPM.h"
#include "Output/Mixers.h"
#include "Control/Pid.h"

namespace Espfc {

enum GyroDlpf {
  GYRO_DLPF_256 = 0x00,
  GYRO_DLPF_188 = 0x01,
  GYRO_DLPF_98  = 0x02,
  GYRO_DLPF_42  = 0x03,
  GYRO_DLPF_20  = 0x04,
  GYRO_DLPF_10  = 0x05,
  GYRO_DLPF_5   = 0x06,
  GYRO_DLPF_EX  = 0x07,
};

enum AccelMode {
  ACCEL_DELAYED   = 0x00,
  ACCEL_GYRO      = 0x01,
};

enum SensorAlign {
  ALIGN_DEFAULT        = 0,
  ALIGN_CW0_DEG        = 1,
  ALIGN_CW90_DEG       = 2,
  ALIGN_CW180_DEG      = 3,
  ALIGN_CW270_DEG      = 4,
  ALIGN_CW0_DEG_FLIP   = 5,
  ALIGN_CW90_DEG_FLIP  = 6,
  ALIGN_CW180_DEG_FLIP = 7,
  ALIGN_CW270_DEG_FLIP = 8,
  ALIGN_CUSTOM         = 9,
};

enum FusionMode {
  FUSION_NONE,
  FUSION_MADGWICK,
  FUSION_MAHONY,
  FUSION_COMPLEMENTARY,
  FUSION_KALMAN,
  FUSION_RTQF,
  FUSION_LERP,
  FUSION_SIMPLE,
  FUSION_EXPERIMENTAL,
  FUSION_MAX,
};

struct FusionConfig
{
  int8_t mode = FUSION_MAHONY;
  uint8_t gain = 50;
  uint8_t gainI = 5;

  static const char * getModeName(FusionMode mode)
  {
    if(mode >= FUSION_MAX) return PSTR("?");
    return getModeNames()[mode];
  }

  static const char ** getModeNames()
  {
    static const char* modeChoices[] = {
      PSTR("NONE"), PSTR("MADGWICK"), PSTR("MAHONY"), PSTR("COMPLEMENTARY"), PSTR("KALMAN"),
      PSTR("RTQF"), PSTR("LERP"), PSTR("SIMPLE"), PSTR("EXPERIMENTAL"),
      NULL };
    return modeChoices;
  }
};

enum FlightMode {
  MODE_ARMED,
  MODE_ANGLE,
  MODE_AIRMODE,
  MODE_BUZZER,
  MODE_FAILSAFE,
  MODE_BLACKBOX,
  MODE_BLACKBOX_ERASE,
  MODE_COUNT,
};

enum ScalerDimension {
  ACT_INNER_P     = 1 << 0,  // 1
  ACT_INNER_I     = 1 << 1,  // 2
  ACT_INNER_D     = 1 << 2,  // 4
  ACT_INNER_F     = 1 << 3,  // 8
  ACT_OUTER_P     = 1 << 4,  // 16
  ACT_OUTER_I     = 1 << 5,  // 32
  ACT_OUTER_D     = 1 << 6,  // 64
  ACT_OUTER_F     = 1 << 7,  // 128
  ACT_AXIS_ROLL   = 1 << 8,  // 256
  ACT_AXIS_PITCH  = 1 << 9,  // 512
  ACT_AXIS_YAW    = 1 << 10, // 1024
  ACT_AXIS_THRUST = 1 << 11, // 2048
  ACT_GYRO_THRUST = 1 << 12, // 4096
};

constexpr size_t SCALER_COUNT = 3;

struct ScalerConfig {
  uint32_t dimension = 0;
  int16_t minScale = 20;
  int16_t maxScale = 400;
  int8_t channel = 0;
};

enum DebugMode {
  DEBUG_NONE,
  DEBUG_CYCLETIME,
  DEBUG_BATTERY,
  DEBUG_GYRO_FILTERED,
  DEBUG_ACCELEROMETER,
  DEBUG_PIDLOOP,
  DEBUG_GYRO_SCALED,
  DEBUG_RC_INTERPOLATION,
  DEBUG_ANGLERATE,
  DEBUG_ESC_SENSOR,
  DEBUG_SCHEDULER,
  DEBUG_STACK,
  DEBUG_ESC_SENSOR_RPM,
  DEBUG_ESC_SENSOR_TMP,
  DEBUG_ALTITUDE,
  DEBUG_FFT,
  DEBUG_FFT_TIME,
  DEBUG_FFT_FREQ,
  DEBUG_RX_FRSKY_SPI,
  DEBUG_RX_SFHSS_SPI,
  DEBUG_GYRO_RAW,
  DEBUG_DUAL_GYRO_RAW,
  DEBUG_DUAL_GYRO_DIFF,
  DEBUG_MAX7456_SIGNAL,
  DEBUG_MAX7456_SPICLOCK,
  DEBUG_SBUS,
  DEBUG_FPORT,
  DEBUG_RANGEFINDER,
  DEBUG_RANGEFINDER_QUALITY,
  DEBUG_LIDAR_TF,
  DEBUG_ADC_INTERNAL,
  DEBUG_RUNAWAY_TAKEOFF,
  DEBUG_SDIO,
  DEBUG_CURRENT_SENSOR,
  DEBUG_USB,
  DEBUG_SMARTAUDIO,
  DEBUG_RTH,
  DEBUG_ITERM_RELAX,
  DEBUG_ACRO_TRAINER,
  DEBUG_RC_SMOOTHING,
  DEBUG_RX_SIGNAL_LOSS,
  DEBUG_RC_SMOOTHING_RATE,
  DEBUG_ANTI_GRAVITY,
  DEBUG_DYN_LPF,
  DEBUG_RX_SPEKTRUM_SPI,
  DEBUG_DSHOT_RPM_TELEMETRY,
  DEBUG_RPM_FILTER,
  DEBUG_D_MIN,
  DEBUG_AC_CORRECTION,
  DEBUG_AC_ERROR,
  DEBUG_DUAL_GYRO_SCALED,
  DEBUG_DSHOT_RPM_ERRORS,
  DEBUG_CRSF_LINK_STATISTICS_UPLINK,
  DEBUG_CRSF_LINK_STATISTICS_PWR,
  DEBUG_CRSF_LINK_STATISTICS_DOWN,
  DEBUG_BARO,
  DEBUG_GPS_RESCUE_THROTTLE_PID,
  DEBUG_DYN_IDLE,
  DEBUG_FF_LIMIT,
  DEBUG_FF_INTERPOLATED,
  DEBUG_BLACKBOX_OUTPUT,
  DEBUG_GYRO_SAMPLE,
  DEBUG_RX_TIMING,
  DEBUG_COUNT,
};

enum Axis {
  AXIS_ROLL,    // x
  AXIS_PITCH,   // y
  AXIS_YAW,     // z
  AXIS_THRUST,  // throttle channel index
  AXIS_AUX_1,
  AXIS_AUX_2,
  AXIS_AUX_3,
  AXIS_AUX_4,
  AXIS_AUX_5,
  AXIS_AUX_6,
  AXIS_AUX_7,
  AXIS_AUX_8,
  AXIS_AUX_9,
  AXIS_AUX_10,
  AXIS_AUX_11,
  AXIS_AUX_12,
  AXIS_COUNT,
  AXIS_COUNT_RP = AXIS_YAW,      // RP axis count
  AXIS_COUNT_RPY = AXIS_THRUST,  // RPY axis count
  AXIS_COUNT_RPYT = AXIS_AUX_1,  // RPYT axis count
};

enum Feature {
  FEATURE_RX_PPM     = 1 << 0,
  FEATURE_RX_SERIAL  = 1 << 3,
  FEATURE_MOTOR_STOP = 1 << 4,
  FEATURE_SOFTSERIAL = 1 << 6,
  FEATURE_GPS        = 1 << 7,
  FEATURE_TELEMETRY  = 1 << 10,
  FEATURE_AIRMODE    = 1 << 22,
  FEATURE_RX_SPI     = 1 << 25,
  FEATURE_DYNAMIC_FILTER = 1 << 29,
};

enum InputInterpolation {
  INPUT_INTERPOLATION_OFF,
  INPUT_INTERPOLATION_DEFAULT,
  INPUT_INTERPOLATION_AUTO,
  INPUT_INTERPOLATION_MANUAL,
};

enum InputFilterType : uint8_t {
  INPUT_INTERPOLATION,
  INPUT_FILTER,
};

constexpr size_t MODEL_NAME_LEN  = 16;
constexpr size_t INPUT_CHANNELS  = AXIS_COUNT;
constexpr size_t OUTPUT_CHANNELS = ESC_CHANNEL_COUNT;
static_assert(ESC_CHANNEL_COUNT == ESPFC_OUTPUT_COUNT, "ESC_CHANNEL_COUNT and ESPFC_OUTPUT_COUNT must be equal");

constexpr size_t RPM_FILTER_MOTOR_MAX = 4;
constexpr size_t RPM_FILTER_HARMONICS_MAX = 3;

enum PinFunction {
#ifdef ESPFC_INPUT
  PIN_INPUT_RX,
#endif
  PIN_OUTPUT_0,
  PIN_OUTPUT_1,
  PIN_OUTPUT_2,
  PIN_OUTPUT_3,
#if ESPFC_OUTPUT_COUNT > 4
  PIN_OUTPUT_4,
#endif
#if ESPFC_OUTPUT_COUNT > 5
  PIN_OUTPUT_5,
#endif
#if ESPFC_OUTPUT_COUNT > 6
  PIN_OUTPUT_6,
#endif
#if ESPFC_OUTPUT_COUNT > 7
  PIN_OUTPUT_7,
#endif
  PIN_BUTTON,
  PIN_BUZZER,
  PIN_LED_BLINK,
#ifdef ESPFC_SERIAL_0
  PIN_SERIAL_0_TX,
  PIN_SERIAL_0_RX,
#endif
#ifdef ESPFC_SERIAL_1
  PIN_SERIAL_1_TX,
  PIN_SERIAL_1_RX,
#endif
#ifdef ESPFC_SERIAL_2
  PIN_SERIAL_2_TX,
  PIN_SERIAL_2_RX,
#endif
#ifdef ESPFC_I2C_0
  PIN_I2C_0_SCL,
  PIN_I2C_0_SDA,
#endif
#ifdef ESPFC_ADC_0
  PIN_INPUT_ADC_0,
#endif
#ifdef ESPFC_ADC_1
  PIN_INPUT_ADC_1,
#endif
#ifdef ESPFC_SPI_0
  PIN_SPI_0_SCK,
  PIN_SPI_0_MOSI,
  PIN_SPI_0_MISO,
#endif
#ifdef ESPFC_SPI_0
  PIN_SPI_CS0,
  PIN_SPI_CS1,
  PIN_SPI_CS2,
#endif
  PIN_COUNT,
};

constexpr size_t ACTUATOR_CONDITIONS = 8;

struct ActuatorCondition
{
  uint8_t id = 0;
  uint8_t ch = AXIS_AUX_1;
  int16_t min = 900;
  int16_t max = 900;
  uint8_t logicMode = 0;
  uint8_t linkId = 0;
};

struct SerialPortConfig
{
  int8_t id;
  int32_t functionMask;
  int32_t baud;
  int32_t blackboxBaud;
};

constexpr size_t BUZZER_MAX_EVENTS = 8;

enum BuzzerEvent {
  BUZZER_SILENCE = 0,             // Silence, see beeperSilence()
  BUZZER_GYRO_CALIBRATED,
  BUZZER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
  BUZZER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
  BUZZER_DISARMING,               // Beep when disarming the board
  BUZZER_ARMING,                  // Beep when arming the board
  BUZZER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
  BUZZER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
  BUZZER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
  BUZZER_GPS_STATUS,              // FIXME **** Disable beeper when connected to USB ****
  BUZZER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
  BUZZER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
  BUZZER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
  BUZZER_READY_BEEP,              // Ring a tone when GPS is locked and ready
  BUZZER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
  BUZZER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
  BUZZER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
  BUZZER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
  BUZZER_USB,                     // Some boards have beeper powered USB connected
  BUZZER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
  BUZZER_CRASH_FLIP_MODE,         // Crash flip mode is active
  BUZZER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
  BUZZER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
  BUZZER_ALL,                     // Turn ON or OFF all beeper conditions
  BUZZER_PREFERENCE,              // Save preferred beeper configuration
  // BUZZER_ALL and BUZZER_PREFERENCE must remain at the bottom of this enum
};

struct BuzzerConfig
{
  int8_t inverted = true;
  int32_t beeperMask;
};

enum PidIndex {
  FC_PID_ROLL,
  FC_PID_PITCH,
  FC_PID_YAW,
  FC_PID_ALT,
  FC_PID_POS,
  FC_PID_POSR,
  FC_PID_NAVR,
  FC_PID_LEVEL,
  FC_PID_MAG,
  FC_PID_VEL,
  FC_PID_ITEM_COUNT,
};

enum BlacboxLogField { // no more than 32, sync with FlightLogFieldSelect_e
  BLACKBOX_FIELD_PID = 0,
  BLACKBOX_FIELD_RC_COMMANDS,
  BLACKBOX_FIELD_SETPOINT,
  BLACKBOX_FIELD_BATTERY,
  BLACKBOX_FIELD_MAG,
  BLACKBOX_FIELD_ALTITUDE,
  BLACKBOX_FIELD_RSSI,
  BLACKBOX_FIELD_GYRO,
  BLACKBOX_FIELD_ACC,
  BLACKBOX_FIELD_DEBUG_LOG,
  BLACKBOX_FIELD_MOTOR,
  BLACKBOX_FIELD_GPS,
  BLACKBOX_FIELD_RPM,
  BLACKBOX_FIELD_GYROUNFILT,
  BLACKBOX_FIELD_COUNT,
};

enum BlackboxLogDevice {
  BLACKBOX_DEV_NONE = 0,
  BLACKBOX_DEV_FLASH = 1,
  BLACKBOX_DEV_SDCARD = 2,
  BLACKBOX_DEV_SERIAL = 3,
};

struct PidConfig
{
  uint8_t P;
  uint8_t I;
  uint8_t D;
  int16_t F;
};

struct InputChannelConfig
{
  int16_t min = 1000;
  int16_t neutral = 1500;
  int16_t max = 2000;
  int8_t map = 0;
  int8_t fsMode = 0;
  int16_t fsValue = 1500;
};

struct InputConfig
{
  int8_t ppmMode = PPM_MODE_NORMAL;
  uint8_t serialRxProvider = SERIALRX_SBUS;

  int16_t maxCheck = 1050;
  int16_t minCheck = 1900;
  int16_t minRc = 885;
  int16_t midRc = 1500;
  int16_t maxRc = 2115;

  int8_t interpolationMode = INPUT_INTERPOLATION_AUTO;
  int8_t interpolationInterval = 26;
  int8_t deadband = 3;

  int8_t filterType = INPUT_FILTER;
  int8_t filterAutoFactor = 50;
  FilterConfig filter{FILTER_PT3, 0};
  FilterConfig filterDerivative{FILTER_PT3, 0};

  uint8_t expo[3] = { 0, 0, 0 };
  uint8_t rate[3] = { 20, 20, 30 };
  uint8_t superRate[3] = { 40, 40,  36 };
  int16_t rateLimit[3] = { 1998, 1998, 1998 };
  int8_t rateType = 3;

  uint8_t rssiChannel = 0;

  InputChannelConfig channel[INPUT_CHANNELS];
};

struct OutputChannelConfig
{
  int16_t min = 1000;
  int16_t neutral = 1500;
  int16_t max = 2000;
  bool reverse = false;
  bool servo = false;
};

struct OutputConfig
{
  int8_t protocol = ESC_PROTOCOL_DISABLED;
  int8_t async = false;
  int8_t dshotTelemetry = false;
  int8_t motorPoles = 14;
  int16_t rate = 480;
  int16_t servoRate = 0;

  int16_t minCommand = 1000;
  int16_t minThrottle = 1070;
  int16_t maxThrottle = 2000;
  int16_t dshotIdle = 550;

  int8_t throttleLimitType = 0;
  int8_t throttleLimitPercent = 100;
  int8_t motorLimit = 100;

  OutputChannelConfig channel[ESPFC_OUTPUT_COUNT];
};

enum DisarmReason {
  DISARM_REASON_ARMING_DISABLED   = 0,
  DISARM_REASON_FAILSAFE          = 1,
  DISARM_REASON_THROTTLE_TIMEOUT  = 2,
  DISARM_REASON_STICKS            = 3,
  DISARM_REASON_SWITCH            = 4,
  DISARM_REASON_CRASH_PROTECTION  = 5,
  DISARM_REASON_RUNAWAY_TAKEOFF   = 6,
  DISARM_REASON_GPS_RESCUE        = 7,
  DISARM_REASON_SERIAL_COMMAND    = 8,
  DISARM_REASON_SYSTEM            = 255,
};

enum ArmingDisabledFlags {
  ARMING_DISABLED_NO_GYRO         = (1 << 0),
  ARMING_DISABLED_FAILSAFE        = (1 << 1),
  ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
  ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
  ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
  ARMING_DISABLED_RUNAWAY_TAKEOFF = (1 << 5),
  ARMING_DISABLED_CRASH_DETECTED  = (1 << 6),
  ARMING_DISABLED_THROTTLE        = (1 << 7),
  ARMING_DISABLED_ANGLE           = (1 << 8),
  ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 9),
  ARMING_DISABLED_NOPREARM        = (1 << 10),
  ARMING_DISABLED_LOAD            = (1 << 11),
  ARMING_DISABLED_CALIBRATING     = (1 << 12),
  ARMING_DISABLED_CLI             = (1 << 13),
  ARMING_DISABLED_CMS_MENU        = (1 << 14),
  ARMING_DISABLED_BST             = (1 << 15),
  ARMING_DISABLED_MSP             = (1 << 16),
  ARMING_DISABLED_PARALYZE        = (1 << 17),
  ARMING_DISABLED_GPS             = (1 << 18),
  ARMING_DISABLED_RESC            = (1 << 19),
  ARMING_DISABLED_RPMFILTER       = (1 << 20),
  ARMING_DISABLED_REBOOT_REQUIRED = (1 << 21),
  ARMING_DISABLED_DSHOT_BITBANG   = (1 << 22),
  ARMING_DISABLED_ACC_CALIBRATION = (1 << 23),
  ARMING_DISABLED_MOTOR_PROTOCOL  = (1 << 24),
  ARMING_DISABLED_ARM_SWITCH      = (1 << 25), // Needs to be the last element, since it's always activated if one of the others is active when arming
};

static constexpr size_t ARMING_DISABLED_FLAGS_COUNT = 25;

struct WirelessConfig
{
  static constexpr size_t MAX_LEN = 32;
  int16_t port = 1111;
  char ssid[MAX_LEN + 1];
  char pass[MAX_LEN + 1];
};

struct FailsafeConfig
{
  uint8_t delay = 4;
  uint8_t killSwitch = 0;
};

struct BlackboxConfig
{
  int8_t dev = 0;
  int16_t pDenom = 32; // 1k
  int32_t fieldsMask = 0xffff;
  int8_t mode = 0;
};

struct DebugConfig
{
  int8_t mode = DEBUG_NONE;
  uint8_t axis = 1;
};

struct RpmFilterConfig
{
  uint8_t harmonics = 3;
  uint8_t minFreq = 100;
  int16_t q = 500;
  uint8_t freqLpf = 150;
  uint8_t weights[RPM_FILTER_HARMONICS_MAX] = {100, 100, 100};
  uint8_t fade = 30;
};

struct VBatConfig
{
  int16_t cellWarning = 350;
  uint8_t scale = 100;
  uint8_t resDiv = 10;
  uint8_t resMult = 1;
  int8_t source = 0;
};

struct IBatConfig
{
  int8_t source = 0;
  int16_t scale = 100;
  int16_t offset = 0;
};

struct GyroConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = GYRO_AUTO;
  int8_t dlpf = GYRO_DLPF_256;
  int8_t align = ALIGN_DEFAULT;
  int16_t bias[3] = { 0, 0, 0 };
  FilterConfig filter{FILTER_PT1, 100};
  FilterConfig filter2{FILTER_PT1, 213};
  FilterConfig filter3{FILTER_FO, 150};
  FilterConfig notch1Filter{FILTER_NOTCH, 0, 0};
  FilterConfig notch2Filter{FILTER_NOTCH, 0, 0};
  FilterConfig dynLpfFilter{FILTER_PT1, 425, 170};
  DynamicFilterConfig dynamicFilter;
  RpmFilterConfig rpmFilter;
};

struct AccelConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = GYRO_AUTO;
  int16_t bias[3] = { 0, 0, 0 };
  FilterConfig filter{FILTER_BIQUAD, 15};
};

struct BaroConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = BARO_NONE;
  FilterConfig filter{FILTER_BIQUAD, 3};
};

struct MagConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = MAG_NONE;
  int8_t align = ALIGN_DEFAULT;
  int16_t offset[3] = { 0, 0, 0 };
  int16_t scale[3] = { 1000, 1000, 1000 };
  FilterConfig filter{FILTER_BIQUAD, 10};
};

struct YawConfig
{
  FilterConfig filter{FILTER_PT1, 90};
};

struct DtermConfig
{
  FilterConfig filter{FILTER_PT1, 128};
  FilterConfig filter2{FILTER_PT1, 128};
  FilterConfig notchFilter{FILTER_NOTCH, 0, 0};
  FilterConfig dynLpfFilter{FILTER_PT1, 145, 60};
  int16_t setpointWeight = 30;
};

struct ItermConfig
{
  int8_t limit = 30;
  int8_t relax = ITERM_RELAX_RP;
  int8_t relaxCutoff = 15;
  bool lowThrottleZeroIterm = true;
};

struct LevelConfig
{
  FilterConfig ptermFilter{FILTER_PT1, 90};
  int8_t angleLimit = 55;
  int16_t rateLimit = 300;
};

struct MixerConfiguration
{
  int8_t type = FC_MIXER_QUADX;
  bool yawReverse = 0;
};

struct ControllerConfig
{
  int8_t tpaScale = 10;
  int16_t tpaBreakpoint = 1650;
};

struct VtxConfig
{
  uint8_t channel = 0x8;
  uint8_t band = 0x1;
  uint8_t power = 0;
  uint8_t lowPowerDisarm = 0;
};

struct GpsConfig
{
  uint8_t minSats = 8;
  uint8_t setHomeOnce = 1;
};

struct LedConfig
{
  uint8_t invert = 0;
};

// persistent data
class ModelConfig
{
  public:
    // inputs and sensors
    GyroConfig gyro;
    AccelConfig accel;
    BaroConfig baro;
    MagConfig mag;
    InputConfig input;
    FailsafeConfig failsafe;
    FusionConfig fusion;
    VBatConfig vbat;
    IBatConfig ibat;
    VtxConfig vtx;
    GpsConfig gps;

    ActuatorCondition conditions[ACTUATOR_CONDITIONS];
    ScalerConfig scaler[SCALER_COUNT];

    // pid controller
    PidConfig pid[FC_PID_ITEM_COUNT] = {
      [FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 24, .F = 72 },  // ROLL
      [FC_PID_PITCH] = { .P = 46, .I = 90, .D = 26, .F = 76 },  // PITCH
      [FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 72 },  // YAW
      [FC_PID_ALT]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // ALT
      [FC_PID_POS]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // POSHOLD_P * 100, POSHOLD_I * 100,
      [FC_PID_POSR]  = { .P =  0, .I =  0, .D =  0, .F =  0 },  // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
      [FC_PID_NAVR]  = { .P =  0, .I =  0, .D =  0, .F =  0 },  // NAV_P * 10, NAV_I * 100, NAV_D * 1000
      [FC_PID_LEVEL] = { .P = 45, .I =  0, .D =  0, .F =  0 },  // LEVEL
      [FC_PID_MAG]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // MAG
      [FC_PID_VEL]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // VEL
    };
    YawConfig yaw;
    LevelConfig level;
    DtermConfig dterm;
    ItermConfig iterm;
    ControllerConfig controller;
    // hardware
    int8_t pin[PIN_COUNT] = {
#ifdef ESPFC_INPUT
      [PIN_INPUT_RX] = ESPFC_INPUT_PIN,
#endif
      [PIN_OUTPUT_0] = ESPFC_OUTPUT_0,
      [PIN_OUTPUT_1] = ESPFC_OUTPUT_1,
      [PIN_OUTPUT_2] = ESPFC_OUTPUT_2,
      [PIN_OUTPUT_3] = ESPFC_OUTPUT_3,
#if ESPFC_OUTPUT_COUNT > 4
      [PIN_OUTPUT_4] = ESPFC_OUTPUT_4,
#endif
#if ESPFC_OUTPUT_COUNT > 5
      [PIN_OUTPUT_5] = ESPFC_OUTPUT_5,
#endif
#if ESPFC_OUTPUT_COUNT > 6
      [PIN_OUTPUT_6] = ESPFC_OUTPUT_6,
#endif
#if ESPFC_OUTPUT_COUNT > 7
      [PIN_OUTPUT_7] = ESPFC_OUTPUT_7,
#endif
      [PIN_BUTTON] = ESPFC_BUTTON_PIN,
      [PIN_BUZZER] = ESPFC_BUZZER_PIN,
      [PIN_LED_BLINK] = ESPFC_LED_PIN,
#ifdef ESPFC_SERIAL_0
      [PIN_SERIAL_0_TX] = ESPFC_SERIAL_0_TX,
      [PIN_SERIAL_0_RX] = ESPFC_SERIAL_0_RX,
#endif
#ifdef ESPFC_SERIAL_1
      [PIN_SERIAL_1_TX] = ESPFC_SERIAL_1_TX,
      [PIN_SERIAL_1_RX] = ESPFC_SERIAL_1_RX,
#endif
#ifdef ESPFC_SERIAL_2
      [PIN_SERIAL_2_TX] = ESPFC_SERIAL_2_TX,
      [PIN_SERIAL_2_RX] = ESPFC_SERIAL_2_RX,
#endif
#ifdef ESPFC_I2C_0
      [PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL,
      [PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA,
#endif
#ifdef ESPFC_ADC_0
      [PIN_INPUT_ADC_0] = ESPFC_ADC_0_PIN,
#endif
#ifdef ESPFC_ADC_1
      [PIN_INPUT_ADC_1] = ESPFC_ADC_1_PIN,
#endif
#ifdef ESPFC_SPI_0
      [PIN_SPI_0_SCK] = ESPFC_SPI_0_SCK,
      [PIN_SPI_0_MOSI] = ESPFC_SPI_0_MOSI,
      [PIN_SPI_0_MISO] = ESPFC_SPI_0_MISO,
      [PIN_SPI_CS0] = ESPFC_SPI_CS_GYRO,
      [PIN_SPI_CS1] = ESPFC_SPI_CS_BARO,
      [PIN_SPI_CS2] = -1,
#endif
    };
    SerialPortConfig serial[SERIAL_UART_COUNT] = {
#ifdef ESPFC_SERIAL_USB
      [SERIAL_USB]    = { .id = SERIAL_ID_USB_VCP, .functionMask = ESPFC_SERIAL_USB_FN, .baud = SERIAL_SPEED_115200, .blackboxBaud = SERIAL_SPEED_NONE },
#endif
#ifdef ESPFC_SERIAL_0
      [SERIAL_UART_0] = { .id = SERIAL_ID_UART_1, .functionMask = ESPFC_SERIAL_0_FN, .baud = ESPFC_SERIAL_0_BAUD, .blackboxBaud = ESPFC_SERIAL_0_BBAUD },
#endif
#ifdef ESPFC_SERIAL_1
      [SERIAL_UART_1] = { .id = SERIAL_ID_UART_2, .functionMask = ESPFC_SERIAL_1_FN, .baud = ESPFC_SERIAL_1_BAUD, .blackboxBaud = ESPFC_SERIAL_1_BBAUD },
#endif
#ifdef ESPFC_SERIAL_2
      [SERIAL_UART_2] = { .id = SERIAL_ID_UART_3, .functionMask = ESPFC_SERIAL_2_FN, .baud = ESPFC_SERIAL_2_BAUD, .blackboxBaud = ESPFC_SERIAL_2_BBAUD },
#endif
#ifdef ESPFC_SERIAL_SOFT_0
      [SERIAL_SOFT_0] = { .id = SERIAL_ID_SOFTSERIAL_1, .functionMask = ESPFC_SERIAL_SOFT_0_FN, .baud = SERIAL_SPEED_115200, .blackboxBaud = SERIAL_SPEED_NONE },
#endif
    };

    LedConfig led;
    BuzzerConfig buzzer;
    WirelessConfig wireless;

    // mixer and outputs
    int8_t customMixerCount = 0;
    MixerEntry customMixes[MIXER_RULE_MAX];
    MixerConfiguration mixer;
    OutputConfig output;
    BlackboxConfig blackbox;
    DebugConfig debug;

    // not classified yet
    int16_t i2cSpeed = 800;
    int8_t loopSync = 8; // MPU 1000Hz
    int8_t mixerSync = 1;
    int32_t featureMask = ESPFC_FEATURE_MASK;
    bool telemetry = 0;
    int32_t telemetryInterval = 1000;
    uint8_t rescueConfigDelay = 30;
    int16_t boardAlignment[3] = {0, 0, 0};
    char modelName[MODEL_NAME_LEN + 1];

    ModelConfig()
    {
      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        input.channel[i].map = i;
        input.channel[i].neutral = input.midRc;
        input.channel[i].fsMode = i <= AXIS_THRUST ? 0 : 1; // auto, hold, set
        input.channel[i].fsValue = i != AXIS_THRUST ? input.midRc : 1000;
      }
      // swap yaw and throttle for AETR
      input.channel[2].map = 3; // replace input 2 with rx channel 3, yaw
      input.channel[3].map = 2; // replace input 3 with rx channel 2, throttle

      // PID controller config (BF default)
      //pid[FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 30, .F = 90 };
      //pid[FC_PID_PITCH] = { .P = 46, .I = 90, .D = 32, .F = 95 };
      //pid[FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 90 };
      //pid[FC_PID_LEVEL] = { .P = 55, .I =  0, .D =  0, .F = 0 };

      wireless.ssid[0] = 0;
      wireless.pass[0] = 0;
      modelName[0] = 0;

      // Custom mixer setup
      // customMixerCount = 5;
      // for(int i = 0; i < customMixerCount; i++) {
      //   customMixes[i] = singlecopterMix[i];
      // }
      // mixer.type = FC_MIXER_CUSTOM;

// development settings
#if !defined(ESPFC_REVISION)
      devPreset();
#endif
    }

    void devPreset()
    {
#ifdef ESPFC_DEV_PRESET_BLACKBOX
      blackbox.dev = BLACKBOX_DEV_SERIAL; // serial
      debug.mode = DEBUG_GYRO_SCALED;
      serial[ESPFC_DEV_PRESET_BLACKBOX].functionMask |= SERIAL_FUNCTION_BLACKBOX;
      serial[ESPFC_DEV_PRESET_BLACKBOX].blackboxBaud = SERIAL_SPEED_250000;
      serial[ESPFC_DEV_PRESET_BLACKBOX].baud = SERIAL_SPEED_250000;
#endif

#ifdef ESPFC_DEV_PRESET_MODES
      conditions[0].id = MODE_ARMED;
      conditions[0].ch = AXIS_AUX_1 + 0; // aux1
      conditions[0].min = 1300;
      conditions[0].max = 2100;
      conditions[0].logicMode = 0;
      conditions[0].linkId = 0;

      conditions[1].id = MODE_ANGLE;
      conditions[1].ch = AXIS_AUX_1 + 0; // aux1
      conditions[1].min = 1700;
      conditions[1].max = 2100;
      conditions[1].logicMode = 0;
      conditions[1].linkId = 0;

      conditions[2].id = MODE_AIRMODE;
      conditions[2].ch = AXIS_AUX_1 + 0; // aux1
      conditions[2].min = 1300;
      conditions[2].max = 2100;
      conditions[2].logicMode = 0;
      conditions[2].linkId = 0;
#endif

#ifdef ESPFC_DEV_PRESET_SCALER
      scaler[0].dimension = (ACT_INNER_P | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[0].channel = AXIS_AUX_1 + 1;
      scaler[1].dimension = (ACT_INNER_I | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[1].channel = AXIS_AUX_1 + 2;
      scaler[2].dimension = (ACT_INNER_D | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[2].channel = AXIS_AUX_1 + 3;
#endif

#ifdef ESPFC_DEV_PRESET_DSHOT
      output.protocol = ESC_PROTOCOL_DSHOT300;
#endif
    }

    void brobot()
    {
      mixer.type = FC_MIXER_GIMBAL;

      pin[PIN_OUTPUT_0] = 14;    // D5 // ROBOT
      pin[PIN_OUTPUT_1] = 12;    // D6 // ROBOT
      pin[PIN_OUTPUT_2] = 15;    // D8 // ROBOT
      pin[PIN_OUTPUT_3] = 0;     // D3 // ROBOT

      //fusionMode = FUSION_SIMPLE;    // ROBOT
      //fusionMode = FUSION_COMPLEMENTARY; // ROBOT
      //accelFilter.freq = 30;        // ROBOT

      iterm.lowThrottleZeroIterm = false; // ROBOT
      iterm.limit = 10; // ROBOT
      dterm.setpointWeight = 0;      // ROBOT
      level.angleLimit = 10;       // deg // ROBOT

      output.protocol = ESC_PROTOCOL_PWM; // ROBOT
      output.rate = 100;    // ROBOT
      output.async = true;  // ROBOT

      output.channel[0].servo = true;   // ROBOT
      output.channel[1].servo = true;   // ROBOT
      output.channel[0].reverse = true; // ROBOT

      scaler[0].dimension = (ACT_INNER_P | ACT_AXIS_PITCH); // ROBOT
      //scaler[1].dimension = (ACT_INNER_P | ACT_AXIS_YAW); // ROBOT
      scaler[1].dimension = (ACT_INNER_I | ACT_AXIS_PITCH); // ROBOT
      scaler[2].dimension = (ACT_INNER_D | ACT_AXIS_PITCH); // ROBOT
    }
};

}

#endif
