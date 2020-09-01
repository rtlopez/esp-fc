#ifndef _ESPFC_MODEL_CONFIG_H_
#define _ESPFC_MODEL_CONFIG_H_

#include "EscDriver.h"
#include "Filter.h"
#include "Device/BusDevice.h"
#include "Device/GyroDevice.h"
#include "Device/MagDevice.h"
#include "Device/BaroDevice.h"
#include "Device/SerialDevice.h"
#include "Device/InputPPM.h"
#include "Output/Mixers.h"

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

enum GyroGain {
  GYRO_FS_250  = 0x00,
  GYRO_FS_500  = 0x01,
  GYRO_FS_1000 = 0x02,
  GYRO_FS_2000 = 0x03
};

enum AccelGain {
  ACCEL_FS_2  = 0x00,
  ACCEL_FS_4  = 0x01,
  ACCEL_FS_8  = 0x02,
  ACCEL_FS_16 = 0x03
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
  ALIGN_CW270_DEG_FLIP = 8
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

class FusionConfig
{
  public:
    int8_t mode;
    uint8_t gain;

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
  MODE_COUNT
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
  ACT_GYRO_THRUST = 1 << 12  // 4096
};

const size_t SCALER_COUNT = 3;

class ScalerConfig {
  public:
    ScalerDimension dimension;
    int16_t minScale;
    int16_t maxScale;
    int8_t channel;
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
  DEBUG_COUNT
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
  AXIS_COUNT
};

enum Feature {
  FEATURE_RX_PPM     = 1 << 0,
  FEATURE_RX_SERIAL  = 1 << 3,
  FEATURE_MOTOR_STOP = 1 << 4,
  FEATURE_SOFTSERIAL = 1 << 6,
  FEATURE_TELEMETRY  = 1 << 10,
  FEATURE_AIRMODE    = 1 << 22,
  FEATURE_DYNAMIC_FILTER = 1 << 29,
};

enum InputInterpolation {
  INPUT_INTERPOLATION_OFF,
  INPUT_INTERPOLATION_DEFAULT,
  INPUT_INTERPOLATION_AUTO,
  INPUT_INTERPOLATION_MANUAL,
};

const size_t MODEL_NAME_LEN  = 16;
const size_t AXES            = 4;
const size_t INPUT_CHANNELS  = AXIS_COUNT;
const size_t OUTPUT_CHANNELS = ESC_CHANNEL_COUNT;

enum PinFunction {
#if defined(ESP8266)
  PIN_INPUT_RX,
  PIN_OUTPUT_0,
  PIN_OUTPUT_1,
  PIN_OUTPUT_2,
  PIN_OUTPUT_3,
  PIN_BUZZER,
  PIN_I2C_0_SCL,
  PIN_I2C_0_SDA,
  PIN_INPUT_ADC_0,
#endif
#if defined(ESP32) || defined(UNIT_TEST)
  PIN_INPUT_RX,
  PIN_OUTPUT_0,
  PIN_OUTPUT_1,
  PIN_OUTPUT_2,
  PIN_OUTPUT_3,
  PIN_OUTPUT_4,
  PIN_OUTPUT_5,
  PIN_OUTPUT_6,
  PIN_OUTPUT_7,
  PIN_BUZZER,
  PIN_SERIAL_0_TX,
  PIN_SERIAL_0_RX,
  PIN_SERIAL_1_TX,
  PIN_SERIAL_1_RX,
  PIN_SERIAL_2_TX,
  PIN_SERIAL_2_RX,
  PIN_I2C_0_SCL,
  PIN_I2C_0_SDA,
  PIN_INPUT_ADC_0,
  PIN_INPUT_ADC_1,
  PIN_SPI_0_SCK,
  PIN_SPI_0_MOSI,
  PIN_SPI_0_MISO,
  PIN_SPI_CS0,
  PIN_SPI_CS1,
  PIN_SPI_CS2,
#endif
  PIN_COUNT
};

#define ACTUATOR_CONDITIONS 8

class ActuatorCondition
{
  public:
    uint8_t id;
    uint8_t ch;
    int16_t min;
    int16_t max;
    uint8_t logicMode;
    uint8_t linkId;
};

class SerialPortConfig
{
  public:
    int8_t id;
    int32_t functionMask;
    int32_t baud;
    int32_t blackboxBaud;
};

#define BUZZER_MAX_EVENTS 8

enum BuzzerEvent {
  BEEPER_SILENCE = 0,             // Silence, see beeperSilence()
  BEEPER_GYRO_CALIBRATED,
  BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
  BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
  BEEPER_DISARMING,               // Beep when disarming the board
  BEEPER_ARMING,                  // Beep when arming the board
  BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
  BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
  BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
  BEEPER_GPS_STATUS,              // FIXME **** Disable beeper when connected to USB ****
  BEEPER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
  BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
  BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
  BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
  BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
  BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
  BEEPER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
  BEEPER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
  BEEPER_USB,                     // Some boards have beeper powered USB connected
  BEEPER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
  BEEPER_CRASH_FLIP_MODE,         // Crash flip mode is active
  BEEPER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
  BEEPER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
  BEEPER_ALL,                     // Turn ON or OFF all beeper conditions
  BEEPER_PREFERENCE,              // Save preferred beeper configuration
  // BEEPER_ALL and BEEPER_PREFERENCE must remain at the bottom of this enum
};

class BuzzerConfig
{
  public:
    int8_t pin;
    int8_t inverted;
    int32_t beeperMask;
};

enum PidIndex {
  PID_ROLL,
  PID_PITCH,
  PID_YAW,
  PID_ALT,
  PID_POS,
  PID_POSR,
  PID_NAVR,
  PID_LEVEL,
  PID_MAG,
  PID_VEL,
  PID_ITEM_COUNT
};

class PidConfig
{
  public:
    uint8_t P;
    uint8_t I;
    uint8_t D;
    int16_t F;
};

class InputChannelConfig
{
  public:
    int16_t min;
    int16_t neutral;
    int16_t max;
    int8_t map;
    int8_t fsMode;
    int16_t fsValue;
};

class InputConfig
{
  public:
    int8_t ppmMode;
    uint8_t serialRxProvider;

    int16_t maxCheck;
    int16_t minCheck;
    int16_t minRc;
    int16_t midRc;
    int16_t maxRc;

    int8_t interpolationMode;
    int8_t interpolationInterval;
    int8_t deadband;

    uint8_t expo[3];
    uint8_t rate[3];
    uint8_t superRate[3];
    int16_t rateLimit[3];
    int8_t rateType;

    uint8_t rssiChannel;

    InputChannelConfig channel[INPUT_CHANNELS];
};

class OutputChannelConfig
{
  public:
    int16_t min;
    int16_t neutral;
    int16_t max;
    bool reverse;
    bool servo;
};

class OutputConfig
{
  public:
    int8_t protocol;
    int16_t async;
    int16_t rate;
    int16_t servoRate;

    int16_t minCommand;
    int16_t minThrottle;
    int16_t maxThrottle;
    int16_t dshotIdle;

    int8_t throttleLimitType = 0;
    int8_t throttleLimitPercent = 100;
    int8_t motorLimit = 100;

    OutputChannelConfig channel[OUTPUT_CHANNELS];
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

static const size_t ARMING_DISABLED_FLAGS_COUNT = 25;

enum WirelessMode {
    WIRELESS_MODE_NULL = 0,  /**< null mode */
    WIRELESS_MODE_STA,       /**< WiFi station mode */
    WIRELESS_MODE_AP,        /**< WiFi soft-AP mode */
    WIRELESS_MODE_APSTA,     /**< WiFi station + soft-AP mode */
    WIRELESS_MODE_MAX
};

class WirelessConfig
{
  public:
    static const size_t MAX_LEN = 32;
    int8_t mode;
    int16_t port;
    char ssid[MAX_LEN + 1];
    char pass[MAX_LEN + 1];
    char ssidAp[MAX_LEN + 1];
    char passAp[MAX_LEN + 1];

    static const char * getModeName(WirelessMode mode)
    {
      if(mode >= WIRELESS_MODE_MAX) return PSTR("?");
      return getModeNames()[mode];
    }

    static const char ** getModeNames()
    {
      static const char* modeChoices[] = { PSTR("OFF"), PSTR("STA"), PSTR("AP"), PSTR("AP_STA"), NULL };
      return modeChoices;
    }
};

class FailsafeConfig
{
  public:
    uint8_t delay;
    uint8_t killSwitch;
};

// persistent data
class ModelConfig
{
  public:
    int8_t gyroBus;
    int8_t gyroDev;
    int8_t gyroDlpf;
    int8_t gyroFsr;
    int8_t gyroSync;
    int8_t gyroAlign;
    FilterConfig gyroFilter;
    FilterConfig gyroFilter2;
    FilterConfig gyroFilter3;
    FilterConfig gyroNotch1Filter;
    FilterConfig gyroNotch2Filter;
    FilterConfig gyroDynLpfFilter;

    int8_t accelBus;
    int8_t accelDev;
    int8_t accelFsr;
    int8_t accelAlign;
    FilterConfig accelFilter;

    int8_t magBus;
    int8_t magDev;
    int8_t magAlign;
    FilterConfig magFilter;

    int8_t baroBus;
    int8_t baroDev;
    FilterConfig baroFilter;

    InputConfig input;
    FailsafeConfig failsafe;

    ActuatorCondition conditions[ACTUATOR_CONDITIONS];

    ScalerConfig scaler[SCALER_COUNT];

    OutputConfig output;

    int8_t mixerType;
    bool yawReverse;

    PidConfig pid[PID_ITEM_COUNT];

    FilterConfig yawFilter;

    FilterConfig dtermFilter;
    FilterConfig dtermFilter2;
    FilterConfig dtermNotchFilter;
    FilterConfig dtermDynLpfFilter;
    FilterConfig levelPtermFilter;

    int16_t dtermSetpointWeight;
    int8_t itermWindupPointPercent;

    int8_t angleLimit;
    int16_t angleRateLimit;

    int8_t loopSync;
    int8_t mixerSync;

    int32_t featureMask;

    bool lowThrottleZeroIterm;

    bool telemetry;
    int32_t telemetryInterval;
    int8_t telemetryPort;

    int8_t blackboxDev;
    int16_t blackboxPdenom;

    SerialPortConfig serial[SERIAL_UART_COUNT];

    FusionConfig fusion;

    int16_t gyroBias[3];
    int16_t accelBias[3];
    int16_t magCalibrationScale[3];
    int16_t magCalibrationOffset[3];

    char modelName[MODEL_NAME_LEN + 1];

    int8_t vbatCellWarning;
    uint8_t vbatScale;
    uint8_t vbatResDiv;
    uint8_t vbatResMult;

    int8_t debugMode;

    BuzzerConfig buzzer;

    int8_t pin[PIN_COUNT];
    int16_t i2cSpeed;
    bool softSerialGuard;
    bool serialRxGuard;
    int8_t tpaScale;
    int16_t tpaBreakpoint;

    int8_t customMixerCount;
    MixerEntry customMixes[MIXER_RULE_MAX];

    WirelessConfig wireless;

    DynamicFilterConfig dynamicFilter;

    ModelConfig()
    {
      pin[PIN_OUTPUT_0] = -1;
      pin[PIN_OUTPUT_1] = -1;
      pin[PIN_OUTPUT_2] = -1;
      pin[PIN_OUTPUT_3] = -1;

      pin[PIN_OUTPUT_0] = 0;     // D3
      pin[PIN_OUTPUT_1] = 14;    // D5
      pin[PIN_OUTPUT_2] = 12;    // D6
      pin[PIN_OUTPUT_3] = 15;    // D8
      pin[PIN_INPUT_RX] = 13;    // D7
      pin[PIN_I2C_0_SCL] = 5;    // D1
      pin[PIN_I2C_0_SDA] = 4;    // D2
      pin[PIN_INPUT_ADC_0] = 17; // A0
      pin[PIN_BUZZER] = 16;      // D0

#if defined(ESP32)
      pin[PIN_INPUT_RX] = 35;
      pin[PIN_OUTPUT_0] = 0;
      pin[PIN_OUTPUT_1] = 2;
      pin[PIN_OUTPUT_2] = 25;
      pin[PIN_OUTPUT_3] = 26;
      pin[PIN_OUTPUT_4] = 27;
      pin[PIN_OUTPUT_5] = 12;
      pin[PIN_OUTPUT_6] = 13;
      pin[PIN_OUTPUT_7] = 14;
      pin[PIN_BUZZER] = 4;
      pin[PIN_SERIAL_0_TX] = 1;
      pin[PIN_SERIAL_0_RX] = 3;
      pin[PIN_SERIAL_1_TX] = 33;
      pin[PIN_SERIAL_1_RX] = 32;
      pin[PIN_SERIAL_2_TX] = 17;
      pin[PIN_SERIAL_2_RX] = 16;
      pin[PIN_I2C_0_SCL] = 22;
      pin[PIN_I2C_0_SDA] = 21;
      pin[PIN_INPUT_ADC_0] = 36;
      pin[PIN_INPUT_ADC_1] = 39;
      pin[PIN_SPI_0_SCK] = 18;
      pin[PIN_SPI_0_MOSI] = 23;
      pin[PIN_SPI_0_MISO] = 19;
      pin[PIN_SPI_CS0] = 5;
      pin[PIN_SPI_CS1] = 15;
      pin[PIN_SPI_CS2] = -1;
#endif
      i2cSpeed = 1000;

      gyroBus = BUS_AUTO;
      gyroDev = GYRO_AUTO;
      gyroAlign = ALIGN_DEFAULT;
      gyroDlpf = GYRO_DLPF_256;
      gyroFsr  = GYRO_FS_2000;
      //gyroSync = 16;
      gyroSync = 8;

      accelBus = BUS_AUTO;
      accelDev = GYRO_AUTO;
      accelAlign = ALIGN_DEFAULT;
      accelFsr = ACCEL_FS_16;

      magBus = BUS_AUTO;
      magDev = MAG_NONE;
      magAlign = ALIGN_DEFAULT;

      baroBus = BUS_AUTO;
      baroDev = BARO_NONE;

      loopSync = 1;
      mixerSync = 1;

      fusion.mode = FUSION_MADGWICK;
      fusion.gain = 50;

      gyroFilter = FilterConfig(FILTER_PT1, 100);
      gyroFilter2 = FilterConfig(FILTER_PT1, 188);
      gyroFilter3 = FilterConfig(FILTER_FIR2, 250); // 0 to off
      gyroDynLpfFilter = FilterConfig(FILTER_PT1, 375, 150);
      gyroNotch1Filter = FilterConfig(FILTER_NOTCH, 0, 0); // off
      gyroNotch2Filter = FilterConfig(FILTER_NOTCH, 0, 0); // off
      dynamicFilter = DynamicFilterConfig(8, 120, 80, 400); // 8%. q:1.2, 80-400 Hz

      dtermFilter = FilterConfig(FILTER_PT1, 90);
      dtermFilter2 = FilterConfig(FILTER_PT1, 113);
      dtermDynLpfFilter = FilterConfig(FILTER_PT1, 128, 53);
      dtermNotchFilter = FilterConfig(FILTER_NOTCH, 0, 0);

      accelFilter = FilterConfig(FILTER_BIQUAD, 10);
      magFilter = FilterConfig(FILTER_BIQUAD, 10);
      yawFilter = FilterConfig(FILTER_PT1, 90);
      levelPtermFilter = FilterConfig(FILTER_PT1, 90);
      baroFilter = FilterConfig(FILTER_BIQUAD, 15);

      telemetry = 0;
      telemetryInterval = 1000;

      softSerialGuard = false;
      serialRxGuard = false;

#if defined(ESP32)
      serial[SERIAL_UART_0].id = SERIAL_ID_UART_1;
      serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
      serial[SERIAL_UART_0].baud = SERIAL_SPEED_115200;
      serial[SERIAL_UART_0].blackboxBaud = SERIAL_SPEED_NONE;

      serial[SERIAL_UART_1].id = SERIAL_ID_UART_2;
      serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_RX_SERIAL;
      serial[SERIAL_UART_1].baud = SERIAL_SPEED_115200;
      serial[SERIAL_UART_1].blackboxBaud = SERIAL_SPEED_NONE;

      serial[SERIAL_UART_2].id = SERIAL_ID_UART_3;
      serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_MSP;
      serial[SERIAL_UART_2].baud = SERIAL_SPEED_115200;
      serial[SERIAL_UART_2].blackboxBaud = SERIAL_SPEED_NONE;
      
      serial[SERIAL_SOFT_0].id = SERIAL_ID_SOFTSERIAL_1; // as soft serial
      serial[SERIAL_SOFT_0].functionMask = SERIAL_FUNCTION_MSP;
      serial[SERIAL_SOFT_0].baud = SERIAL_SPEED_115200;
      serial[SERIAL_SOFT_0].blackboxBaud = SERIAL_SPEED_NONE;

      //serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_TELEMETRY_FRSKY;
      //serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_BLACKBOX;
      //serial[SERIAL_UART_2].blackboxBaud = SERIAL_SPEED_250000;
      //serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_TELEMETRY_FRSKY;
#elif defined(ESP8266)
      serial[SERIAL_UART_0].id = SERIAL_ID_UART_1;
      serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
      serial[SERIAL_UART_0].baud = SERIAL_SPEED_115200;
      serial[SERIAL_UART_0].blackboxBaud = SERIAL_SPEED_NONE;

      serial[SERIAL_UART_1].id = SERIAL_ID_UART_2;
      serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_NONE;
      serial[SERIAL_UART_1].baud = SERIAL_SPEED_115200;
      serial[SERIAL_UART_1].blackboxBaud = SERIAL_SPEED_NONE;

      serial[SERIAL_SOFT_0].id = SERIAL_ID_SOFTSERIAL_1; // present as soft serial
      serial[SERIAL_SOFT_0].functionMask = SERIAL_FUNCTION_NONE;
      serial[SERIAL_SOFT_0].baud = SERIAL_SPEED_115200;
      serial[SERIAL_SOFT_0].blackboxBaud = SERIAL_SPEED_NONE;
#endif

      // output config
      output.minCommand  = 1000;
      output.minThrottle = 1050;
      output.maxThrottle = 2000;
      output.dshotIdle = 450;
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        output.channel[i].servo = false;
        output.channel[i].reverse = false;
        output.channel[i].min = 1000;
        output.channel[i].max = 2000;
        output.channel[i].neutral = 1500;
      }

      mixerType = MIXER_QUADX;
      yawReverse = 0;

      output.protocol = ESC_PROTOCOL_DISABLED;
      //output.rate = 2000; // max 500 for PWM, 2000 for Oneshot125
      output.rate = 480;    // max 500 for PWM, 2000 for Oneshot125
      //output.async = true;
      output.async = false;
      output.servoRate = 0; // default 50, 0 to disable

      // input config
      input.ppmMode = PPM_MODE_NORMAL;
      input.minCheck = 1050;
      input.maxCheck = 1900;
      input.minRc = 885;
      input.midRc = 1500;
      input.maxRc = 2115;
      input.rssiChannel = 0;
      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        input.channel[i].map = i;
        input.channel[i].min = 1000;
        input.channel[i].neutral = input.midRc;
        input.channel[i].max = 2000;
        input.channel[i].fsMode = i <= AXIS_THRUST ? 0 : 1; // auto, hold, set
        input.channel[i].fsValue = i != AXIS_THRUST ? input.midRc : 1000;
      }
      // swap yaw and throttle for AETR
      input.channel[2].map = 3; // replace input 2 with rx channel 3, yaw
      input.channel[3].map = 2; // replace input 3 with rx channel 2, throttle

      input.rateType = 0; // betaflight

      input.rate[AXIS_ROLL] = 70;
      input.expo[AXIS_ROLL] = 0;
      input.superRate[AXIS_ROLL] = 80;
      input.rateLimit[AXIS_ROLL] = 1998;

      input.rate[AXIS_PITCH] = 70;
      input.expo[AXIS_PITCH] = 0;
      input.superRate[AXIS_PITCH] = 80;
      input.rateLimit[AXIS_PITCH] = 1998;

      input.rate[AXIS_YAW] = 120;
      input.expo[AXIS_YAW] = 0;
      input.superRate[AXIS_YAW] = 50;
      input.rateLimit[AXIS_YAW] = 1998;

      input.interpolationMode = INPUT_INTERPOLATION_AUTO; // mode
      //input.interpolationMode = INPUT_INTERPOLATION_MANUAL; // mode
      input.interpolationInterval = 26;
      input.deadband = 3; // us

      failsafe.delay = 4;
      failsafe.killSwitch = 0;

      // PID controller config
      pid[PID_ROLL]  = { .P = 42, .I = 85, .D = 30, .F = 90 };
      pid[PID_PITCH] = { .P = 46, .I = 90, .D = 32, .F = 95 };
      pid[PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 90 };
      pid[PID_LEVEL] = { .P = 55, .I =  0, .D =  0, .F = 0 };

      pid[PID_ALT]   = { .P = 0, .I =  0, .D =  0, .F = 0 };
      pid[PID_POS]   = { .P = 0, .I =  0, .D =  0, .F = 0 };  // POSHOLD_P * 100, POSHOLD_I * 100,
      pid[PID_POSR]  = { .P = 0, .I =  0, .D =  0, .F = 0 };  // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
      pid[PID_NAVR]  = { .P = 0, .I =  0, .D =  0, .F = 0 };  // NAV_P * 10, NAV_I * 100, NAV_D * 1000
      pid[PID_MAG]   = { .P = 0, .I =  0, .D =  0, .F = 0 };
      pid[PID_VEL]   = { .P = 0, .I =  0, .D =  0, .F = 0 };

      itermWindupPointPercent = 30;
      dtermSetpointWeight = 30;

      angleLimit = 55;  // deg
      angleRateLimit = 300;  // deg

    #if defined(ESP8266)
      featureMask = FEATURE_RX_PPM | FEATURE_DYNAMIC_FILTER;
    #elif defined(ESP32)
      featureMask = FEATURE_RX_SERIAL | FEATURE_SOFTSERIAL | FEATURE_DYNAMIC_FILTER;
    #endif

      input.serialRxProvider = SERIALRX_SBUS;

      lowThrottleZeroIterm = true;

      tpaScale = 10;
      tpaBreakpoint = 1650;

      for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        conditions[i].id = 0;
        conditions[i].ch = AXIS_AUX_1 + 0;
        conditions[i].min = 900;
        conditions[i].max = 900;
      }

      //conditions[0].id = MODE_ARMED;
      //conditions[0].ch = AXIS_AUX_1 + 0;
      //conditions[0].min = 1700;
      //conditions[0].max = 2100;

      //conditions[1].id = MODE_ANGLE;
      //conditions[1].ch = AXIS_AUX_1 + 0; // aux1
      //conditions[1].min = 1900;
      //conditions[1].max = 2100;

      //conditions[2].id = MODE_AIRMODE;
      //conditions[2].ch = 0; // aux1
      //conditions[2].min = (1700 - 900) / 25;
      //conditions[2].max = (2100 - 900) / 25;

      //conditions[3].id = MODE_FAILSAFE;
      //conditions[3].ch = 1; // aux1
      //conditions[3].min = (1700 - 900) / 25;
      //conditions[3].max = (2100 - 900) / 25;

      //conditions[4].id = MODE_BUZZER;
      //conditions[4].ch = 2; // aux1
      //conditions[4].min = (1700 - 900) / 25;
      //conditions[4].max = (2100 - 900) / 25;

      // actuator config - pid scaling
      scaler[0].dimension = (ScalerDimension)(0);
      scaler[0].channel = 0;
      scaler[0].minScale = 20; //%
      scaler[0].maxScale = 400;

      scaler[1].dimension = (ScalerDimension)(0);
      scaler[1].channel = 0;
      scaler[1].minScale = 20; //%
      scaler[1].maxScale = 400;

      scaler[2].dimension = (ScalerDimension)(0);
      scaler[2].channel = 0;
      scaler[2].minScale = 20; //%
      scaler[2].maxScale = 200;

      // default callibration values
      gyroBias[0] = 0;
      gyroBias[1] = 0;
      gyroBias[2] = 0;
      accelBias[0] = 0;
      accelBias[1] = 0;
      accelBias[2] = 0;
      magCalibrationOffset[0] = 0;
      magCalibrationOffset[1] = 0;
      magCalibrationOffset[2] = 0;
      magCalibrationScale[0] = 1000;
      magCalibrationScale[1] = 1000;
      magCalibrationScale[2] = 1000;

      vbatScale = 100;
      vbatResDiv = 16;
      vbatResMult = 1;
      vbatCellWarning = 35;

      buzzer.inverted = true;

      wireless.mode = WIRELESS_MODE_NULL;
      wireless.ssid[0] = 0;
      wireless.pass[0] = 0;
      wireless.ssidAp[0] = 0;
      wireless.passAp[0] = 0;
      wireless.port = 1111;

      modelName[0] = 0;

// development settings
#if !defined(ESPFC_REVISION)
  #if defined(ESP8266)
      output.protocol = ESC_PROTOCOL_DSHOT150;
      serial[SERIAL_UART_1].id = SERIAL_ID_UART_2;
      serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_BLACKBOX;
      serial[SERIAL_UART_1].blackboxBaud = SERIAL_SPEED_250000;
  #elif defined(ESP32)
      output.protocol = ESC_PROTOCOL_DSHOT300;
      serial[SERIAL_UART_2].id = SERIAL_ID_UART_3;
      serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_BLACKBOX;
      serial[SERIAL_UART_2].blackboxBaud = SERIAL_SPEED_250000;
  #endif

      quad();
#endif
    }

    void quad()
    {
      debugMode = DEBUG_GYRO_SCALED;
      blackboxDev = 3; // serial
      blackboxPdenom = 32; // 1kHz

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

      scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[0].channel = AXIS_AUX_1 + 1;
      scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[1].channel = AXIS_AUX_1 + 2;
      scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[2].channel = AXIS_AUX_1 + 3;
    }

    void brobot()
    {
      mixerType = MIXER_GIMBAL;

      pin[PIN_OUTPUT_0] = 14;    // D5 // ROBOT
      pin[PIN_OUTPUT_1] = 12;    // D6 // ROBOT
      pin[PIN_OUTPUT_2] = 15;    // D8 // ROBOT
      pin[PIN_OUTPUT_3] = 0;     // D3 // ROBOT

      //fusionMode = FUSION_SIMPLE;    // ROBOT
      //fusionMode = FUSION_COMPLEMENTARY; // ROBOT
      //accelFilter.freq = 30;        // ROBOT

      lowThrottleZeroIterm = false; // ROBOT
      itermWindupPointPercent = 10; // ROBOT
      dtermSetpointWeight = 0;      // ROBOT
      angleLimit = 10;       // deg // ROBOT

      output.protocol = ESC_PROTOCOL_PWM; // ROBOT
      output.rate = 100;    // ROBOT
      output.async = true;  // ROBOT

      output.channel[0].servo = true;   // ROBOT
      output.channel[1].servo = true;   // ROBOT
      output.channel[0].reverse = true; // ROBOT

      scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_PITCH); // ROBOT
      //scaler[1].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_YAW); // ROBOT
      scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_PITCH); // ROBOT
      scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_PITCH); // ROBOT
    }
};

}

#endif
