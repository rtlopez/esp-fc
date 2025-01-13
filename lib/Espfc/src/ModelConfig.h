#ifndef _ESPFC_MODEL_CONFIG_H_
#define _ESPFC_MODEL_CONFIG_H_

#include "Target/Target.h"
#include "EscDriver.h"
#include "Filter.h"
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

class FusionConfig
{
  public:
    int8_t mode;
    uint8_t gain;
    uint8_t gainI;

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
  INPUT_FILTER
};

const size_t MODEL_NAME_LEN  = 16;
const size_t AXES            = 4;
const size_t AXES_RPY        = 3;
const size_t INPUT_CHANNELS  = AXIS_COUNT;
const size_t OUTPUT_CHANNELS = ESC_CHANNEL_COUNT;
static_assert(ESC_CHANNEL_COUNT == ESPFC_OUTPUT_COUNT, "ESC_CHANNEL_COUNT and ESPFC_OUTPUT_COUNT must be equal");

const size_t RPM_FILTER_MOTOR_MAX = 4;
const size_t RPM_FILTER_HARMONICS_MAX = 3;

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
  PIN_BUZZER,
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

class BuzzerConfig
{
  public:
    int8_t inverted;
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
  FC_PID_ITEM_COUNT
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

    int8_t filterType;
    int8_t filterAutoFactor;
    FilterConfig filter;
    FilterConfig filterDerivative;

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
    int8_t async;
    int8_t dshotTelemetry;
    int8_t motorPoles;
    int16_t rate;
    int16_t servoRate;

    int16_t minCommand;
    int16_t minThrottle;
    int16_t maxThrottle;
    int16_t dshotIdle;

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

static const size_t ARMING_DISABLED_FLAGS_COUNT = 25;

class WirelessConfig
{
  public:
    static const size_t MAX_LEN = 32;
    int16_t port;
    char ssid[MAX_LEN + 1];
    char pass[MAX_LEN + 1];
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
    int8_t gyroAlign;
    FilterConfig gyroFilter;
    FilterConfig gyroFilter2;
    FilterConfig gyroFilter3;
    FilterConfig gyroNotch1Filter;
    FilterConfig gyroNotch2Filter;
    FilterConfig gyroDynLpfFilter;

    int8_t accelBus;
    int8_t accelDev;
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

    PidConfig pid[FC_PID_ITEM_COUNT];

    FilterConfig yawFilter;

    FilterConfig dtermFilter;
    FilterConfig dtermFilter2;
    FilterConfig dtermNotchFilter;
    FilterConfig dtermDynLpfFilter;
    FilterConfig levelPtermFilter;

    int16_t dtermSetpointWeight;
    int8_t itermLimit;
    int8_t itermRelax;
    int8_t itermRelaxCutoff;

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
    int32_t blackboxFieldsDisabledMask;
    int8_t blackboxMode;

    SerialPortConfig serial[SERIAL_UART_COUNT];

    FusionConfig fusion;

    int16_t gyroBias[3];
    int16_t accelBias[3];
    int16_t magCalibrationScale[3];
    int16_t magCalibrationOffset[3];

    char modelName[MODEL_NAME_LEN + 1];

    int16_t vbatCellWarning;
    uint8_t vbatScale;
    uint8_t vbatResDiv;
    uint8_t vbatResMult;
    int8_t vbatSource;

    int8_t ibatSource;
    int16_t ibatScale;
    int16_t ibatOffset;

    int8_t debugMode;
    uint8_t debugAxis;

    BuzzerConfig buzzer;

    int8_t pin[PIN_COUNT];
    int16_t i2cSpeed;
    int8_t tpaScale;
    int16_t tpaBreakpoint;

    int8_t customMixerCount;
    MixerEntry customMixes[MIXER_RULE_MAX];

    WirelessConfig wireless;

    DynamicFilterConfig dynamicFilter;

    uint8_t rpmFilterHarmonics;
    uint8_t rpmFilterMinFreq;
    int16_t rpmFilterQ;
    uint8_t rpmFilterFreqLpf;
    uint8_t rpmFilterWeights[RPM_FILTER_HARMONICS_MAX];
    uint8_t rpmFilterFade;

    uint8_t rescueConfigDelay = 30;

    int16_t boardAlignment[3] = {0, 0, 0};

    ModelConfig()
    {
#ifdef ESPFC_INPUT
      pin[PIN_INPUT_RX] = ESPFC_INPUT_PIN;
#endif
      pin[PIN_OUTPUT_0] = ESPFC_OUTPUT_0;
      pin[PIN_OUTPUT_1] = ESPFC_OUTPUT_1;
      pin[PIN_OUTPUT_2] = ESPFC_OUTPUT_2;
      pin[PIN_OUTPUT_3] = ESPFC_OUTPUT_3;
#if ESPFC_OUTPUT_COUNT > 4
      pin[PIN_OUTPUT_4] = ESPFC_OUTPUT_4;
#endif
#if ESPFC_OUTPUT_COUNT > 5
      pin[PIN_OUTPUT_5] = ESPFC_OUTPUT_5;
#endif
#if ESPFC_OUTPUT_COUNT > 6
      pin[PIN_OUTPUT_6] = ESPFC_OUTPUT_6;
#endif
#if ESPFC_OUTPUT_COUNT > 7
      pin[PIN_OUTPUT_7] = ESPFC_OUTPUT_7;
#endif
      pin[PIN_BUZZER] = ESPFC_BUZZER_PIN;
#ifdef ESPFC_SERIAL_0
      pin[PIN_SERIAL_0_TX] = ESPFC_SERIAL_0_TX;
      pin[PIN_SERIAL_0_RX] = ESPFC_SERIAL_0_RX;
#endif
#ifdef ESPFC_SERIAL_1
      pin[PIN_SERIAL_1_TX] = ESPFC_SERIAL_1_TX;
      pin[PIN_SERIAL_1_RX] = ESPFC_SERIAL_1_RX;
#endif
#ifdef ESPFC_SERIAL_2
      pin[PIN_SERIAL_2_TX] = ESPFC_SERIAL_2_TX;
      pin[PIN_SERIAL_2_RX] = ESPFC_SERIAL_2_RX;
#endif
#ifdef ESPFC_I2C_0
      pin[PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL;
      pin[PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA;
#endif
#ifdef ESPFC_ADC_0
      pin[PIN_INPUT_ADC_0] = ESPFC_ADC_0_PIN;
#endif
#ifdef ESPFC_ADC_1
      pin[PIN_INPUT_ADC_1] = ESPFC_ADC_1_PIN;
#endif
#ifdef ESPFC_SPI_0
      pin[PIN_SPI_0_SCK] = ESPFC_SPI_0_SCK;
      pin[PIN_SPI_0_MOSI] = ESPFC_SPI_0_MOSI;
      pin[PIN_SPI_0_MISO] = ESPFC_SPI_0_MISO;
#endif
#ifdef ESPFC_SPI_0
      pin[PIN_SPI_CS0] = ESPFC_SPI_CS_GYRO;
      pin[PIN_SPI_CS1] = ESPFC_SPI_CS_BARO;
      pin[PIN_SPI_CS2] = -1;
#endif
      i2cSpeed = 800;

      gyroBus = BUS_AUTO;
      gyroDev = GYRO_AUTO;
      gyroAlign = ALIGN_DEFAULT;
      gyroDlpf = GYRO_DLPF_256;

      loopSync = 8; // MPU 1000Hz
      mixerSync = 1;

      accelBus = BUS_AUTO;
      accelDev = GYRO_AUTO;
      accelAlign = ALIGN_DEFAULT;

      magBus = BUS_AUTO;
      magDev = MAG_NONE;
      magAlign = ALIGN_DEFAULT;

      baroBus = BUS_AUTO;
      baroDev = BARO_NONE;

      fusion.mode = FUSION_MAHONY;
      fusion.gain = 50;
      fusion.gainI = 5;

      // BF x 0.85
      gyroDynLpfFilter = FilterConfig(FILTER_PT1, 425, 170);
      gyroFilter = FilterConfig(FILTER_PT1, 100);
      gyroFilter2 = FilterConfig(FILTER_PT1, 213);
      dynamicFilter = DynamicFilterConfig(4, 300, 80, 400); // 8%. q:3.0, 80-400 Hz

      dtermDynLpfFilter = FilterConfig(FILTER_PT1, 145, 60);
      dtermFilter = FilterConfig(FILTER_PT1, 128);
      dtermFilter2 = FilterConfig(FILTER_PT1, 128);

      rpmFilterHarmonics = 3;
      rpmFilterMinFreq = 100;
      rpmFilterQ = 500;
      rpmFilterFade = 30;
      rpmFilterWeights[0] = 100;
      rpmFilterWeights[1] = 100;
      rpmFilterWeights[2] = 100;
      rpmFilterFreqLpf = 150;

      gyroFilter3 = FilterConfig(FILTER_FO, 150);
      gyroNotch1Filter = FilterConfig(FILTER_NOTCH, 0, 0); // off
      gyroNotch2Filter = FilterConfig(FILTER_NOTCH, 0, 0); // off
      dtermNotchFilter = FilterConfig(FILTER_NOTCH, 0, 0); // off

      accelFilter = FilterConfig(FILTER_BIQUAD, 15);
      magFilter = FilterConfig(FILTER_BIQUAD, 10);
      yawFilter = FilterConfig(FILTER_PT1, 90);
      levelPtermFilter = FilterConfig(FILTER_PT1, 90);
      baroFilter = FilterConfig(FILTER_BIQUAD, 10);

      telemetry = 0;
      telemetryInterval = 1000;

#ifdef ESPFC_SERIAL_0
      serial[SERIAL_UART_0].id = SERIAL_ID_UART_1;
      serial[SERIAL_UART_0].functionMask = ESPFC_SERIAL_0_FN;
      serial[SERIAL_UART_0].baud = ESPFC_SERIAL_0_BAUD;
      serial[SERIAL_UART_0].blackboxBaud = ESPFC_SERIAL_0_BBAUD;
#endif
#ifdef ESPFC_SERIAL_1
      serial[SERIAL_UART_1].id = SERIAL_ID_UART_2;
      serial[SERIAL_UART_1].functionMask = ESPFC_SERIAL_1_FN;
      serial[SERIAL_UART_1].baud = ESPFC_SERIAL_1_BAUD;
      serial[SERIAL_UART_1].blackboxBaud = ESPFC_SERIAL_1_BBAUD;
#endif
#ifdef ESPFC_SERIAL_2
      serial[SERIAL_UART_2].id = SERIAL_ID_UART_3;
      serial[SERIAL_UART_2].functionMask = ESPFC_SERIAL_2_FN;
      serial[SERIAL_UART_2].baud = ESPFC_SERIAL_2_BAUD;
      serial[SERIAL_UART_2].blackboxBaud = ESPFC_SERIAL_2_BBAUD;
#endif
#ifdef ESPFC_SERIAL_USB
      serial[SERIAL_USB].id = SERIAL_ID_USB_VCP;
      serial[SERIAL_USB].functionMask = ESPFC_SERIAL_USB_FN;
      serial[SERIAL_USB].baud = SERIAL_SPEED_115200;
      serial[SERIAL_USB].blackboxBaud = SERIAL_SPEED_NONE;
#endif
#ifdef ESPFC_SERIAL_SOFT_0
      serial[SERIAL_SOFT_0].id = SERIAL_ID_SOFTSERIAL_1;
      serial[SERIAL_SOFT_0].functionMask = ESPFC_SERIAL_SOFT_0_FN;
      serial[SERIAL_SOFT_0].baud = SERIAL_SPEED_115200;
      serial[SERIAL_SOFT_0].blackboxBaud = SERIAL_SPEED_NONE;
#endif

      // output config
      output.minCommand  = 1000;
      output.minThrottle = 1070;
      output.maxThrottle = 2000;
      output.dshotIdle = 550;
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        output.channel[i].servo = false;
        output.channel[i].reverse = false;
        output.channel[i].min = 1000;
        output.channel[i].max = 2000;
        output.channel[i].neutral = 1500;
      }

      mixerType = FC_MIXER_QUADX;
      yawReverse = 0;

      output.protocol = ESC_PROTOCOL_DISABLED;
      //output.rate = 2000; // max 500 for PWM, 2000 for Oneshot125
      output.rate = 480;    // max 500 for PWM, 2000 for Oneshot125
      //output.async = true;
      output.async = false;
      output.servoRate = 0; // default 50, 0 to disable
      output.dshotTelemetry = false;
      output.motorPoles = 14;

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

      input.rateType = 3; // actual

      input.rate[AXIS_ROLL] = 20;
      input.expo[AXIS_ROLL] = 0;
      input.superRate[AXIS_ROLL] = 40;
      input.rateLimit[AXIS_ROLL] = 1998;

      input.rate[AXIS_PITCH] = 20;
      input.expo[AXIS_PITCH] = 0;
      input.superRate[AXIS_PITCH] = 40;
      input.rateLimit[AXIS_PITCH] = 1998;

      input.rate[AXIS_YAW] = 30;
      input.expo[AXIS_YAW] = 0;
      input.superRate[AXIS_YAW] = 36;
      input.rateLimit[AXIS_YAW] = 1998;

      input.filterType = INPUT_FILTER;
      input.filterAutoFactor = 50;
      input.filter = FilterConfig(FILTER_PT3, 0); // 0: auto
      input.filterDerivative = FilterConfig(FILTER_PT3, 0); // 0: auto

      input.interpolationMode = INPUT_INTERPOLATION_AUTO; // mode
      input.interpolationInterval = 26;
      input.deadband = 3; // us

      failsafe.delay = 4;
      failsafe.killSwitch = 0;

      // PID controller config (BF default)
      //pid[FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 30, .F = 90 };
      //pid[FC_PID_PITCH] = { .P = 46, .I = 90, .D = 32, .F = 95 };
      //pid[FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 90 };
      //pid[FC_PID_LEVEL] = { .P = 55, .I =  0, .D =  0, .F = 0 };

      // PID controller config (ESPFC default)
      pid[FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 24, .F = 72 };
      pid[FC_PID_PITCH] = { .P = 46, .I = 90, .D = 26, .F = 76 };
      pid[FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 72 };
      pid[FC_PID_LEVEL] = { .P = 45, .I =  0, .D =  0, .F = 0 };

      pid[FC_PID_ALT]   = { .P = 0, .I =  0, .D =  0, .F = 0 };
      pid[FC_PID_POS]   = { .P = 0, .I =  0, .D =  0, .F = 0 };  // POSHOLD_P * 100, POSHOLD_I * 100,
      pid[FC_PID_POSR]  = { .P = 0, .I =  0, .D =  0, .F = 0 };  // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
      pid[FC_PID_NAVR]  = { .P = 0, .I =  0, .D =  0, .F = 0 };  // NAV_P * 10, NAV_I * 100, NAV_D * 1000
      pid[FC_PID_MAG]   = { .P = 0, .I =  0, .D =  0, .F = 0 };
      pid[FC_PID_VEL]   = { .P = 0, .I =  0, .D =  0, .F = 0 };

      itermLimit = 30;
      itermRelax = ITERM_RELAX_RP;
      itermRelaxCutoff = 15;
      dtermSetpointWeight = 30;

      angleLimit = 55;  // deg
      angleRateLimit = 300;  // deg

      featureMask = ESPFC_FEATURE_MASK;

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
        conditions[i].logicMode = 0;
        conditions[i].linkId = 0;
      }

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

      vbatSource = 0;
      vbatScale = 100;
      vbatResDiv = 10;
      vbatResMult = 1;
      vbatCellWarning = 350;

      ibatSource = 0;
      ibatScale = 100;
      ibatOffset = 0;

      buzzer.inverted = true;

      wireless.ssid[0] = 0;
      wireless.pass[0] = 0;
      wireless.port = 1111;

      modelName[0] = 0;

      debugMode = DEBUG_NONE;
      debugAxis = 1;
      blackboxDev = 0;
      blackboxPdenom = 32; // 1kHz
      blackboxFieldsDisabledMask = 0;
      blackboxMode = 0;

// development settings
#if !defined(ESPFC_REVISION)
      devPreset();
#endif
    }

    void devPreset()
    {
#ifdef ESPFC_DEV_PRESET_BLACKBOX
      blackboxDev = 3; // serial
      debugMode = DEBUG_GYRO_SCALED;
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
      scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[0].channel = AXIS_AUX_1 + 1;
      scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[1].channel = AXIS_AUX_1 + 2;
      scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[2].channel = AXIS_AUX_1 + 3;
#endif

#ifdef ESPFC_DEV_PRESET_DSHOT
      output.protocol = ESC_PROTOCOL_DSHOT300;
#endif
    }

    void brobot()
    {
      mixerType = FC_MIXER_GIMBAL;

      pin[PIN_OUTPUT_0] = 14;    // D5 // ROBOT
      pin[PIN_OUTPUT_1] = 12;    // D6 // ROBOT
      pin[PIN_OUTPUT_2] = 15;    // D8 // ROBOT
      pin[PIN_OUTPUT_3] = 0;     // D3 // ROBOT

      //fusionMode = FUSION_SIMPLE;    // ROBOT
      //fusionMode = FUSION_COMPLEMENTARY; // ROBOT
      //accelFilter.freq = 30;        // ROBOT

      lowThrottleZeroIterm = false; // ROBOT
      itermLimit = 10; // ROBOT
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
