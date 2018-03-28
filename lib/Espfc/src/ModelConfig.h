#ifndef _ESPFC_MODEL_CONFIG_H_
#define _ESPFC_MODEL_CONFIG_H_

#include <Arduino.h>
#include "EscDriver.h"

#define EEPROM_VERSION_NUM 0x04

#define USE_SOFT_SERIAL

namespace Espfc {

enum GyroDlpf {
  GYRO_DLPF_256 = 0x00,
  GYRO_DLPF_188 = 0x01,
  GYRO_DLPF_98  = 0x02,
  GYRO_DLPF_42  = 0x03,
  GYRO_DLPF_20  = 0x04,
  GYRO_DLPF_10  = 0x05,
  GYRO_DLPF_5   = 0x06
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

enum MagGain {
  MAG_GAIN_1370 = 0x00,
  MAG_GAIN_1090 = 0x01,
  MAG_GAIN_820  = 0x02,
  MAG_GAIN_660  = 0x03,
  MAG_GAIN_440  = 0x04,
  MAG_GAIN_390  = 0x05,
  MAG_GAIN_330  = 0x06,
  MAG_GAIN_220  = 0x07,
};

enum MagRate {
  MAG_RATE_3    = 0x00,
  MAG_RATE_7P5  = 0x01,
  MAG_RATE_15   = 0x02,
  MAG_RATE_30   = 0x03,
  MAG_RATE_75   = 0x04,
};

enum MagAvg {
  MAG_AVERAGING_1 = 0x00,
  MAG_AVERAGING_2 = 0x01,
  MAG_AVERAGING_4 = 0x02,
  MAG_AVERAGING_8 = 0x03
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
  FUSION_COMPLEMENTARY,
  FUSION_KALMAN,
  FUSION_RTQF,
  FUSION_LERP,
  FUSION_SIMPLE,
  FUSION_EXPERIMENTAL,
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
  ACT_INNER_P     = 1 << 0, // 1
  ACT_INNER_I     = 1 << 1, // 2
  ACT_INNER_D     = 1 << 2, // 4
  ACT_OUTER_P     = 1 << 3, // 8
  ACT_OUTER_I     = 1 << 4, // 16
  ACT_OUTER_D     = 1 << 5, // 32
  ACT_AXIS_ROLL   = 1 << 6, // 64
  ACT_AXIS_PITCH  = 1 << 7, // 128
  ACT_AXIS_YAW    = 1 << 8, // 256
  ACT_AXIS_THRUST = 1 << 9, // 512
  ACT_GYRO_THRUST = 1 << 10 // 1024
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
  DEBUG_GYRO,
  DEBUG_ACCELEROMETER,
  DEBUG_MIXER,
  DEBUG_AIRMODE,
  DEBUG_PIDLOOP,
  DEBUG_NOTCH,
  DEBUG_RC_INTERPOLATION,
  DEBUG_VELOCITY,
  DEBUG_DTERM_FILTER,
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
  DEBUG_FRSKY_D_RX,
  DEBUG_COUNT
};

enum SerialSpeed {
  SERIAL_SPEED_NONE   =      0,
  SERIAL_SPEED_9600   =   9600,
  SERIAL_SPEED_19200  =  19200,
  SERIAL_SPEED_38400  =  38400,
  SERIAL_SPEED_57600  =  57600,
  SERIAL_SPEED_115200 = 115200,
  SERIAL_SPEED_230400 = 230400,
  SERIAL_SPEED_250000 = 250000,
  SERIAL_SPEED_500000 = 500000
};

enum SerialSpeedIndex {
  SERIAL_SPEED_INDEX_AUTO = 0,
  SERIAL_SPEED_INDEX_9600,
  SERIAL_SPEED_INDEX_19200,
  SERIAL_SPEED_INDEX_38400,
  SERIAL_SPEED_INDEX_57600,
  SERIAL_SPEED_INDEX_115200,
  SERIAL_SPEED_INDEX_230400,
  SERIAL_SPEED_INDEX_250000,
  SERIAL_SPEED_INDEX_500000 = 10
};

enum SerialPort {
  SERIAL_UART_0,
  SERIAL_UART_1,
#if defined(ESP32)
  SERIAL_UART_2,
#endif
#if defined(ESP8266)
  SERIAL_SOFT_0,
#endif
  SERIAL_UART_COUNT
};

enum SerialFunction {
  SERIAL_FUNCTION_NONE                = 0,
  SERIAL_FUNCTION_MSP                 = (1 << 0),  // 1
  SERIAL_FUNCTION_GPS                 = (1 << 1),  // 2
  SERIAL_FUNCTION_TELEMETRY_FRSKY     = (1 << 2),  // 4
  SERIAL_FUNCTION_TELEMETRY_HOTT      = (1 << 3),  // 8
  SERIAL_FUNCTION_TELEMETRY_LTM       = (1 << 4),  // 16
  SERIAL_FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
  SERIAL_FUNCTION_RX_SERIAL           = (1 << 6),  // 64
  SERIAL_FUNCTION_BLACKBOX            = (1 << 7),  // 128
  SERIAL_FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
  SERIAL_FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
  SERIAL_FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
  SERIAL_FUNCTION_TELEMETRY_IBUS      = (1 << 12), // 4096
  SERIAL_FUNCTION_VTX_TRAMP           = (1 << 13), // 8192
  SERIAL_FUNCTION_RCDEVICE            = (1 << 14), // 16384
};

enum AccelDev {
  ACCEL_DEFAULT = 0,
  ACCEL_NONE    = 1,
  ACCEL_MPU6050 = 3,
  ACCEL_MPU6000 = 7,
  ACCEL_MPU6500 = 8,
  ACCEL_MPU9250 = 9
};

enum MagDev {
  MAG_DEFAULT = 0,
  MAG_NONE    = 1,
  MAG_HMC5883 = 2,
  MAG_AK8975  = 3,
  MAG_AK8963  = 4
};

enum BaroDev {
  BARO_DEFAULT = 0,
  BARO_NONE    = 1,
  BARO_BMP085  = 2,
  BARO_MS5611  = 3,
  BARO_BMP280  = 4
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
  AXIS_COUNT
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

enum Feature {
  FEATURE_RX_PPM     = 1 << 0,
  FEATURE_RX_SERIAL  = 1 << 3,
  FEATURE_MOTOR_STOP = 1 << 4,
  FEATURE_SOFTSERIAL = 1 << 6,
  FEATURE_TELEMETRY  = 1 << 10,
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
#if defined(ESP32)
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
  PIN_SPI_0_CS0,
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
};

class SerialPortConfig
{
  public:
    int8_t id;
    int16_t functionMask;
    int8_t baudIndex;
    int8_t blackboxBaudIndex;
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

enum FilterType {
  FILTER_PT1,
  FILTER_BIQUAD,
  FILTER_FIR,
  FILTER_NOTCH,
  FILTER_NONE,
};

enum BiquadFilterType {
  BIQUAD_FILTER_LPF,
  BIQUAD_FILTER_NOTCH,
  BIQUAD_FILTER_BPF
};

class FilterConfig
{
  public:
    int8_t type;
    int16_t freq;
    int16_t cutoff;
};

class PidConfig
{
  public:
    uint8_t P;
    uint8_t I;
    uint8_t D;
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

    int16_t minCommand;
    int16_t minThrottle;
    int16_t maxThrottle;
    int16_t dshotIdle;

    OutputChannelConfig channel[OUTPUT_CHANNELS];
};

enum MixerType {
  MIXER_TRI = 1,
  MIXER_QUADP = 2,
  MIXER_QUADX = 3,
  MIXER_BICOPTER = 4,
  MIXER_GIMBAL = 5,
  MIXER_Y6 = 6,
  MIXER_HEX6 = 7,
  MIXER_FLYING_WING = 8,
  MIXER_Y4 = 9,
  MIXER_HEX6X = 10,
  MIXER_OCTOX8 = 11,
  MIXER_OCTOFLATP = 12,
  MIXER_OCTOFLATX = 13,
  MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
  MIXER_HELI_120_CCPM = 15,
  MIXER_HELI_90_DEG = 16,
  MIXER_VTAIL4 = 17,
  MIXER_HEX6H = 18,
  MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
  MIXER_DUALCOPTER = 20,
  MIXER_SINGLECOPTER = 21,
  MIXER_ATAIL4 = 22,
  MIXER_CUSTOM = 23,
  MIXER_CUSTOM_AIRPLANE = 24,
  MIXER_CUSTOM_TRI = 25,
  MIXER_QUADX_1234 = 26,
};

enum MixerSource {
  MIXER_SOURCE_NULL,
  MIXER_SOURCE_ROLL,
  MIXER_SOURCE_PITCH,
  MIXER_SOURCE_YAW,
  MIXER_SOURCE_THRUST,
  MIXER_SOURCE_RC_ROLL,
  MIXER_SOURCE_RC_PITCH,
  MIXER_SOURCE_RC_YAW,
  MIXER_SOURCE_RC_THRUST,
  MIXER_SOURCE_RC_AUX1,
  MIXER_SOURCE_RC_AUX2,
  MIXER_SOURCE_RC_AUX3,
  MIXER_SOURCE_MAX,
};

static const size_t MIXER_RULE_MAX = 64;

class MixerEntry {
  public:
    MixerEntry(): src(MIXER_SOURCE_NULL), dst(0), rate(0) {}
    MixerEntry(int8_t s, int8_t d, int16_t r): src(s), dst(d), rate(r) {}
    int8_t src;
    int8_t dst;
    int16_t rate;
};

enum ArmingDisabledFlags {
  ARMING_DISABLED_NO_GYRO         = (1 << 0),
  ARMING_DISABLED_FAILSAFE        = (1 << 1),
  ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
  ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
  ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
  ARMING_DISABLED_RUNAWAY_TAKEOFF = (1 << 5),
  ARMING_DISABLED_THROTTLE        = (1 << 6),
  ARMING_DISABLED_ANGLE           = (1 << 7),
  ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 8),
  ARMING_DISABLED_NOPREARM        = (1 << 9),
  ARMING_DISABLED_LOAD            = (1 << 10),
  ARMING_DISABLED_CALIBRATING     = (1 << 11),
  ARMING_DISABLED_CLI             = (1 << 12),
  ARMING_DISABLED_CMS_MENU        = (1 << 13),
  ARMING_DISABLED_OSD_MENU        = (1 << 14),
  ARMING_DISABLED_BST             = (1 << 15),
  ARMING_DISABLED_MSP             = (1 << 16),
  ARMING_DISABLED_ARM_SWITCH      = (1 << 17), // Needs to be the last element, since it's always activated if one of the others is active when arming
};

static const size_t ARMING_DISABLED_FLAGS_COUNT = 18;

// persistent data
class ModelConfig
{
  public:
    int8_t gyroDev;
    int8_t gyroDlpf;
    int8_t gyroFsr;
    int8_t gyroSync;
    int8_t gyroAlign;
    FilterConfig gyroFilter;
    FilterConfig gyroNotch1Filter;
    FilterConfig gyroNotch2Filter;

    int8_t accelDev;
    int8_t accelFsr;
    int8_t accelMode;
    int8_t accelAlign;
    FilterConfig accelFilter;

    int8_t magDev;
    int8_t magSampleRate;
    int8_t magFsr;
    int8_t magAvr;
    int8_t magAlign;
    FilterConfig magFilter;

    int8_t baroDev;

    InputConfig input;

    ActuatorCondition conditions[ACTUATOR_CONDITIONS];

    ScalerConfig scaler[SCALER_COUNT];

    OutputConfig output;

    int8_t mixerType;
    bool yawReverse;

    PidConfig pid[PID_ITEM_COUNT];

    FilterConfig yawFilter;

    FilterConfig dtermFilter;
    FilterConfig dtermNotchFilter;
    FilterConfig levelPtermFilter;

    uint8_t dtermSetpointWeight;
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

    int8_t fusionMode;

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

    ModelConfig()
    {
#if defined(ESP8266)
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
#endif
#if defined(ESP32)
      pin[PIN_INPUT_RX] = 35;
      pin[PIN_OUTPUT_0] = 32;
      pin[PIN_OUTPUT_1] = 33;
      pin[PIN_OUTPUT_2] = 25;
      pin[PIN_OUTPUT_3] = 26;
      pin[PIN_OUTPUT_4] = 27;
      pin[PIN_OUTPUT_5] = 14;
      pin[PIN_OUTPUT_6] = 12;
      pin[PIN_OUTPUT_7] = 13;
      pin[PIN_BUZZER] = 15;
      pin[PIN_SERIAL_0_TX] = 1;
      pin[PIN_SERIAL_0_RX] = 3;
      //pin[PIN_SERIAL_0_TX] = 17;
      //pin[PIN_SERIAL_0_RX] = 16;
      pin[PIN_SERIAL_1_TX] = 2;
      pin[PIN_SERIAL_1_RX] = 4;
      pin[PIN_SERIAL_2_TX] = 17;
      pin[PIN_SERIAL_2_RX] = 16;
      //pin[PIN_SERIAL_2_TX] = 1;
      //pin[PIN_SERIAL_2_RX] = 3;
      pin[PIN_I2C_0_SCL] = 22;
      pin[PIN_I2C_0_SDA] = 21;
      pin[PIN_INPUT_ADC_0] = 36;
      pin[PIN_INPUT_ADC_1] = 39;
      pin[PIN_SPI_0_SCK] = 18;
      pin[PIN_SPI_0_MOSI] = 23;
      pin[PIN_SPI_0_MISO] = 19;
      pin[PIN_SPI_0_CS0] = 5;
#endif
      i2cSpeed = 1000;

      gyroDev = ACCEL_MPU6050;
      accelDev = ACCEL_MPU6050;
      gyroAlign = ALIGN_DEFAULT;
      accelAlign = ALIGN_DEFAULT;
      magAlign = ALIGN_DEFAULT;

      accelMode = ACCEL_DELAYED;
      gyroDlpf = GYRO_DLPF_256;
      gyroFsr  = GYRO_FS_2000;
      accelFsr = ACCEL_FS_8;
      //gyroSync = 16;
      gyroSync = 8;

      magDev = MAG_NONE;
      magSampleRate = MAG_RATE_75;
      magAvr = MAG_AVERAGING_1;

      baroDev = BARO_NONE;

      loopSync = 1;
      mixerSync = 1;

      fusionMode = FUSION_MADGWICK;

      gyroFilter.type = FILTER_PT1;
      gyroFilter.freq = 90;
      gyroNotch1Filter.type = FILTER_NOTCH;
      gyroNotch1Filter.cutoff = 70;
      gyroNotch1Filter.freq = 150;
      gyroNotch2Filter.type = FILTER_NOTCH;
      gyroNotch2Filter.cutoff = 150;
      gyroNotch2Filter.freq = 300;

      accelFilter.type = FILTER_BIQUAD;
      accelFilter.freq = 15;

      magFilter.type = FILTER_BIQUAD;
      magFilter.freq = 15;

      dtermFilter.type = FILTER_BIQUAD;
      dtermFilter.freq = 100;

      dtermNotchFilter.type = FILTER_BIQUAD;
      dtermNotchFilter.cutoff = 70;
      dtermNotchFilter.freq = 130;

      yawFilter.type = FILTER_PT1;
      yawFilter.freq = 90;

      levelPtermFilter.type = FILTER_PT1;
      levelPtermFilter.freq = 70;

      telemetry = 0;
      telemetryInterval = 1000;

      debugMode = DEBUG_NONE;
      softSerialGuard = false;
      serialRxGuard = false;

      blackboxDev = 3;
      blackboxPdenom = 32;

#if defined(ESP32)
      serial[SERIAL_UART_0].id = SERIAL_UART_0;
      serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
      //serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_TELEMETRY_FRSKY;
      serial[SERIAL_UART_0].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_UART_0].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;

      serial[SERIAL_UART_1].id = SERIAL_UART_1;
      serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_NONE;
      serial[SERIAL_UART_1].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_UART_1].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;

      serial[SERIAL_UART_2].id = SERIAL_UART_2;
      serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_TELEMETRY_FRSKY;
      //serial[SERIAL_UART_2].functionMask = SERIAL_FUNCTION_MSP;
      serial[SERIAL_UART_2].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_UART_2].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;
#endif

#if defined(ESP8266)
      serial[SERIAL_UART_0].id = SERIAL_UART_0;
      serial[SERIAL_UART_0].functionMask = SERIAL_FUNCTION_MSP;
      serial[SERIAL_UART_0].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_UART_0].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;

      serial[SERIAL_UART_1].id = SERIAL_UART_1;
      serial[SERIAL_UART_1].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_BLACKBOX;
      serial[SERIAL_UART_1].blackboxBaudIndex = SERIAL_SPEED_INDEX_250000;
      //serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_TELEMETRY_FRSKY;
      //serial[SERIAL_UART_1].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;

      serial[SERIAL_SOFT_0].id = 30; // present as soft serial
      serial[SERIAL_SOFT_0].functionMask = SERIAL_FUNCTION_NONE;
      serial[SERIAL_SOFT_0].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_SOFT_0].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;
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

      //output.protocol = ESC_PROTOCOL_PWM;
      output.protocol = ESC_PROTOCOL_ONESHOT125;
      //output.protocol = ESC_PROTOCOL_MULTISHOT;
      //output.protocol = ESC_PROTOCOL_BRUSHED;
      //output.rate = 2000;    // max 500 for PWM, 2000 for Oneshot125
      output.rate = 480;    // max 500 for PWM, 2000 for Oneshot125
      output.async = false;
      //output.async = true;

      // input config
      input.ppmMode = RISING;
      input.minCheck = 1050;
      input.maxCheck = 1900;
      input.minRc = 850;
      input.midRc = 1500;
      input.maxRc = 2150;
      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        input.channel[i].map = i;
        input.channel[i].min = 1000;
        input.channel[i].neutral = input.midRc;
        input.channel[i].max = 2000;
        input.channel[i].fsMode = i <= AXIS_THRUST ? 0 : 2;
        input.channel[i].fsValue = i >= AXIS_THRUST ? 1000 : 1500;
      }
      // swap yaw and throttle for AETR
      input.channel[2].map = 3; // replace input 2 with rx channel 3, yaw
      input.channel[3].map = 2; // replace input 3 with rx channel 2, throttle

      input.rate[AXIS_ROLL] = 70;
      input.expo[AXIS_ROLL] = 0;
      input.superRate[AXIS_ROLL] = 80;

      input.rate[AXIS_PITCH] = 70;
      input.expo[AXIS_PITCH] = 0;
      input.superRate[AXIS_PITCH] = 80;

      input.rate[AXIS_YAW] = 120;
      input.expo[AXIS_YAW] = 0;
      input.superRate[AXIS_YAW] = 50;

      //input.interpolationMode = INPUT_INTERPOLATION_AUTO; // mode
      input.interpolationMode = INPUT_INTERPOLATION_MANUAL; // mode
      input.interpolationInterval = 26;
      input.deadband = 3; // us

      // PID controller config
      pid[PID_ROLL]  = { .P = 40,  .I = 40, .D = 30 };
      pid[PID_PITCH] = { .P = 58,  .I = 50, .D = 35 };
      pid[PID_YAW]   = { .P = 70,  .I = 45, .D = 10 };
      pid[PID_LEVEL] = { .P = 55,  .I = 0,  .D = 0 };

      itermWindupPointPercent = 30;
      dtermSetpointWeight = 30;

      angleLimit = 55;  // deg
      angleRateLimit = 300;  // deg

      featureMask = FEATURE_RX_PPM | FEATURE_MOTOR_STOP;
      //featureMask = FEATURE_RX_SERIAL | FEATURE_SOFTSERIAL | FEATURE_MOTOR_STOP;

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

      conditions[0].id = MODE_ARMED;
      conditions[0].ch = AXIS_AUX_1 + 0;
      conditions[0].min = 1700;
      conditions[0].max = 2100;

      conditions[1].id = MODE_ANGLE;
      conditions[1].ch = AXIS_AUX_1 + 0; // aux1
      conditions[1].min = 1900;
      conditions[1].max = 2100;

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
      scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
      scaler[0].channel = 5;
      scaler[0].minScale = 25; //%
      scaler[0].maxScale = 400;

      scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
      scaler[1].channel = 6;
      scaler[1].minScale = 25; //%
      scaler[1].maxScale = 400;

      scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
      scaler[2].channel = 7;
      scaler[2].minScale = 25; //%
      scaler[2].maxScale = 400;

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
