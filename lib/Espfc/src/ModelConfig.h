#ifndef _ESPFC_MODEL_CONFIG_H_
#define _ESPFC_MODEL_CONFIG_H_

#include <Arduino.h>
#include "EscDriver.h"

#define EEPROM_VERSION_NUM 0x0B

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
  ACCEL_OFF       = 0x00,
  ACCEL_DELAYED   = 0x01,
  ACCEL_GYRO      = 0x02,
  ACCEL_GYRO_FIFO = 0x03
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

enum ModelFrame {
  FRAME_UNCONFIGURED  = 0x00,
  FRAME_QUAD_X        = 0x01,
  FRAME_BALANCE_ROBOT = 0x02,
  FRAME_DIRECT        = 0x03,
  FRAME_GIMBAL        = 0x04
};

enum ActuatorConfig {
  ACT_INNER_P     = 1 << 0,
  ACT_INNER_I     = 1 << 1,
  ACT_INNER_D     = 1 << 2,
  ACT_OUTER_P     = 1 << 3,
  ACT_OUTER_I     = 1 << 4,
  ACT_OUTER_D     = 1 << 5,
  ACT_AXIS_ROLL   = 1 << 6,
  ACT_AXIS_PITCH  = 1 << 7,
  ACT_AXIS_YAW    = 1 << 8,
  ACT_AXIS_THRUST = 1 << 9,
  ACT_GYRO_THRUST = 1 << 10
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
  FEATURE_MOTOR_STOP = 1 << 4,
  FEATURE_TELEMETRY  = 1 << 10
};

enum InputInterpolation {
  INPUT_INTERPOLATION_OFF,
  INPUT_INTERPOLATION_DEFAULT,
  INPUT_INTERPOLATION_AUTO,
  INPUT_INTERPOLATION_MANUAL
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
  PIN_SERIAL_0_TX,
  PIN_SERIAL_0_RX,
  PIN_SERIAL_1_TX,
  PIN_SERIAL_1_RX,
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
    uint8_t min;
    uint8_t max;
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

class OutputConfig
{

};

// persistent data
class ModelConfig
{
  public:
    int8_t gyroDev;
    int8_t gyroDlpf;
    int8_t gyroFsr;
    int8_t gyroSync;
    int16_t gyroSampleRate;
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

    int32_t actuatorConfig[3];
    int8_t actuatorChannels[3];
    int16_t actuatorMin[3];
    int16_t actuatorMax[3];

    int8_t mixerType;

    int16_t outputMinThrottle;
    int16_t outputMaxThrottle;
    int16_t outputMinCommand;

    int16_t outputMin[OUTPUT_CHANNELS];
    int16_t outputNeutral[OUTPUT_CHANNELS];
    int16_t outputMax[OUTPUT_CHANNELS];

    int8_t outputProtocol;
    int16_t outputAsync;
    int16_t outputRate;
    int8_t yawReverse;

    PidConfig pid[PID_ITEM_COUNT];

    FilterConfig yawFilter;

    FilterConfig dtermFilter;
    FilterConfig dtermNotchFilter;

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
    bool fusionDelay;

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

    ModelConfig()
    {
#if defined(ESP8266)
      pin[PIN_OUTPUT_0] = 0;     // D3
      pin[PIN_OUTPUT_1] = 14;    // D5
      pin[PIN_OUTPUT_2] = 12;    // D6
      pin[PIN_OUTPUT_3] = 15;    // D8
      pin[PIN_INPUT_RX] = 13;    // D7
      pin[PIN_SERIAL_0_TX] = 1;  // TX0
      pin[PIN_SERIAL_0_RX] = 3;  // RX0
      pin[PIN_SERIAL_1_TX] = 2;  // TX1, D4
      pin[PIN_SERIAL_1_RX] = -1;  // RX1, unavailable
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

      gyroDev = ACCEL_MPU6050;
      accelDev = ACCEL_MPU6050;
      gyroAlign = ALIGN_DEFAULT;
      accelAlign = ALIGN_DEFAULT;
      magAlign = ALIGN_DEFAULT;

      accelMode = ACCEL_GYRO;
      gyroDlpf = GYRO_DLPF_256;
      gyroFsr  = GYRO_FS_2000;
      accelFsr = ACCEL_FS_8;
      gyroSync = 16;

      magDev = MAG_NONE;
      magSampleRate = MAG_RATE_75;
      magAvr = MAG_AVERAGING_1;

      baroDev = BARO_NONE;

      loopSync = 1;
      mixerSync = 1;

      fusionMode = FUSION_MADGWICK;
      fusionDelay = 0;

      gyroFilter.type = FILTER_PT1;
      gyroFilter.freq = 70;
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
      dtermFilter.freq = 50;

      dtermNotchFilter.type = FILTER_BIQUAD;
      dtermNotchFilter.cutoff = 70;
      dtermNotchFilter.freq = 130;

      yawFilter.type = FILTER_PT1;
      yawFilter.freq = 50;

      telemetry = 0;
      telemetryInterval = 1000;

      debugMode = DEBUG_NONE;

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
      //serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_TELEMETRY_FRSKY;
      serial[SERIAL_UART_1].functionMask = SERIAL_FUNCTION_BLACKBOX;
      serial[SERIAL_UART_1].baudIndex = SERIAL_SPEED_INDEX_115200;
      //serial[SERIAL_UART_1].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;
      serial[SERIAL_UART_1].blackboxBaudIndex = SERIAL_SPEED_INDEX_250000;

      serial[SERIAL_SOFT_0].id = 30; // present as soft serial
      serial[SERIAL_SOFT_0].functionMask = SERIAL_FUNCTION_NONE;
      serial[SERIAL_SOFT_0].baudIndex = SERIAL_SPEED_INDEX_115200;
      serial[SERIAL_SOFT_0].blackboxBaudIndex = SERIAL_SPEED_INDEX_AUTO;
#endif

      // output config
      outputMinCommand  = 1000;
      outputMinThrottle = 1050;
      outputMaxThrottle = 2000;
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        outputMin[i] = outputMinThrottle;
        outputMax[i] = outputMaxThrottle;
        outputNeutral[i] = (outputMin[i] + outputMax[i] + 1) / 2;
      }

      mixerType = FRAME_QUAD_X;
      yawReverse = 0;

      outputProtocol = ESC_PROTOCOL_ONESHOT125;
      outputRate = 480;    // max 500 for PWM, 2000 for Oneshot125
      outputAsync = false;

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
        input.channel[i].fsMode = 2;
        input.channel[i].fsValue = 1500;
      }
      // swap yaw and throttle for AETR
      input.channel[2].map = 3; // replace input 2 with rx channel 3, yaw
      input.channel[3].map = 2; // replace input 3 with rx channel 2, throttle

      input.channel[AXIS_ROLL].fsMode = 0;
      input.channel[AXIS_PITCH].fsMode = 0;
      input.channel[AXIS_YAW].fsMode = 0;
      input.channel[AXIS_THRUST].fsMode = 0;
      input.channel[AXIS_THRUST].fsValue = 1000;

      for(size_t i = AXIS_ROLL; i <= AXIS_PITCH; i++)
      {
        input.rate[i] = 70;
        input.expo[i] = 0;
        input.superRate[i] = 80;
      }
      input.rate[AXIS_YAW] = 120;
      input.expo[AXIS_YAW] = 0;
      input.superRate[AXIS_YAW] = 50;

      input.interpolationMode = INPUT_INTERPOLATION_MANUAL; // mode
      input.interpolationInterval = 26;
      input.deadband = 3; // us

      // PID controller config
      pid[PID_ROLL]  = { .P = 45,  .I = 45, .D = 15 };
      pid[PID_PITCH] = { .P = 70,  .I = 55, .D = 20 };
      pid[PID_YAW]   = { .P = 125, .I = 75, .D = 5 };
      pid[PID_LEVEL] = { .P = 30,  .I = 0,  .D = 0 };

      itermWindupPointPercent = 30;
      dtermSetpointWeight = 50;

      angleLimit = 50;  // deg
      angleRateLimit = 300;  // deg

      featureMask = FEATURE_RX_PPM | FEATURE_MOTOR_STOP;

      lowThrottleZeroIterm = true;

      conditions[0].id = MODE_ARMED;
      conditions[0].ch = 0; // aux1
      conditions[0].min = (1700 - 900) / 25;
      conditions[0].max = (2100 - 900) / 25;

      conditions[1].id = MODE_ANGLE;
      conditions[1].ch = 0; // aux1
      conditions[1].min = (1700 - 900) / 25;
      conditions[1].max = (2100 - 900) / 25;

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
      actuatorConfig[0] = ACT_INNER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      actuatorChannels[0] = 5;
      actuatorMin[0] = 25;  // %
      actuatorMax[0] = 400;

      actuatorConfig[1] = ACT_INNER_I | ACT_AXIS_PITCH | ACT_AXIS_ROLL | ACT_AXIS_YAW;
      actuatorChannels[1] = 6;
      actuatorMin[1] = 25;
      actuatorMax[1] = 400;

      actuatorConfig[2] = ACT_INNER_D | ACT_AXIS_PITCH | ACT_AXIS_ROLL;
      actuatorChannels[2] = 7;
      actuatorMin[2] = 25;
      actuatorMax[2] = 400;

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
      mixerType = FRAME_BALANCE_ROBOT;  // ROBOT

      pin[PIN_OUTPUT_0] = 14;    // D5 // ROBOT
      pin[PIN_OUTPUT_1] = 12;    // D6 // ROBOT
      pin[PIN_OUTPUT_2] = 15;    // D8 // ROBOT
      pin[PIN_OUTPUT_3] = 0;     // D3 // ROBOT

      //fusionMode = FUSION_SIMPLE; // ROBOT
      //fusionMode = FUSION_COMPLEMENTARY; // ROBOT

      //accelFilter.freq = 30; // ROBOT

      lowThrottleZeroIterm = false; // ROBOT
      itermWindupPointPercent = 10; // ROBOT
      dtermSetpointWeight = 0; // ROBOT
      angleLimit = 8;  // deg // ROBOT

      outputProtocol = ESC_PROTOCOL_PWM; // ROBOT
      outputRate = 100;    // ROBOT
      outputAsync = true; // ROBOT

      outputMinThrottle = 1000; // ROBOT
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        outputMin[i] = outputMinThrottle;
        outputMax[i] = outputMaxThrottle;
        outputNeutral[i] = (outputMin[i] + outputMax[i] + 1) / 2;
      }
      outputMin[0] = outputMaxThrottle; // ROBOT
      outputMax[0] = outputMinThrottle; // ROBOT

      actuatorConfig[0] = ACT_INNER_P | ACT_AXIS_PITCH; // ROBOT
      //actuatorConfig[1] = ACT_INNER_P | ACT_AXIS_YAW; // ROBOT
      actuatorConfig[1] = ACT_INNER_I | ACT_AXIS_PITCH; // ROBOT
      actuatorConfig[2] = ACT_INNER_D | ACT_AXIS_PITCH; // ROBOT
    }
};

}

#endif
