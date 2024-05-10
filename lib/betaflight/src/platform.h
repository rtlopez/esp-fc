#pragma once

#define USE_BLACKBOX
#define USE_ACC
#define USE_MAG
#define USE_BARO
#define USE_DYN_LPF
#define USE_D_MIN
#define USE_DYN_NOTCH_FILTER
#define USE_ITERM_RELAX
#define USE_DSHOT_TELEMETRY
#define USE_RPM_FILTER

#include <stdbool.h>
#include <stdint.h>
#include <printf.h>
#include <stddef.h>

#if defined(ESP32)
#define USE_FLASHFS
#include "esp_partition.h"
#endif

#if defined(ESP8266)
#define ESPFC_TARGET "ESP8266"
#elif defined(ESP32S3)
#define ESPFC_TARGET "ESP32S3"
#elif defined(ESP32S2)
#define ESPFC_TARGET "ESP32S2"
#elif defined(ESP32C3)
#define ESPFC_TARGET "ESP32C3"
#elif defined(ESP32)
#define ESPFC_TARGET "ESP32"
#elif defined(ARCH_RP2040)
#define ESPFC_TARGET "RP2040"
#elif defined(UNIT_TEST)
#define ESPFC_TARGET "UNIT"
#else
  #error "Unsupported platform"
#endif

#ifndef ESPFC_REVISION
#define ESPFC_REVISION 0000000
#endif

#ifndef ESPFC_VERSION
#define ESPFC_VERSION v0.0.0
#endif

#define MAX_SUPPORTED_MOTORS 8
#define MAX_SUPPORTED_SERVOS 8
#define PID_PROCESS_DENOM_DEFAULT       1

#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_VERSION_MAJOR            4  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            4  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

#ifdef __cplusplus
extern "C" {
#endif

extern const char * const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char * const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char * const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char * const buildTime;  // "HH:MM:SS"

extern const char * pidnames;
extern const char * const targetVersion;
extern const char * flightControllerIdentifier;
extern const char * boardIdentifier;

/* UTILS START */
#ifndef MIN
#define MIN(a,b) ((a > b) ? (b) : (a))
#endif
#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)
#define CONCAT2(_1,_2) CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)  CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)
#define XYZ_AXIS_COUNT 3
#define DEBUG16_VALUE_COUNT 8
#define DEBUG_SET(mode, index, value) {if (debugMode == (mode)) {debug[(index)] = (value);}}

#define LOG2_8BIT(v)  (8 - 90/(((v)/4+14)|1) - 2/((v)/2+1))
#define LOG2_16BIT(v) (8*((v)>255) + LOG2_8BIT((v) >>8*((v)>255)))
#define LOG2_32BIT(v) (16*((v)>65535L) + LOG2_16BIT((v)*1L >>16*((v)>65535L)))
#define LOG2_64BIT(v) (32*((v)/2L>>31 > 0) + LOG2_32BIT((v)*1L >>16*((v)/2L>>31 > 0) >>16*((v)/2L>>31 > 0)))
#define LOG2(v) LOG2_64BIT(v)

#ifdef UNIT_TEST
#define STATIC_UNIT_TESTED
#else
#define STATIC_UNIT_TESTED
#endif

#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)
#define UNUSED(v) ((void)v)
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define STATIC_ASSERT(condition, name) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ] __attribute__((unused))

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count);
uint32_t castFloatBytesToInt(float f);
uint32_t zigzagEncode(int32_t value);

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

unsigned long millis(void);
unsigned long micros(void);

#if __GNUC__ > 6
#define FALLTHROUGH __attribute__ ((fallthrough))
#else
#define FALLTHROUGH do {} while(0)
#endif

#define FORMATTED_DATE_TIME_BUFSIZE 30
/* UTILS END */

/* PARAMETER GROUP START */
#define PG_BLACKBOX_CONFIG 5
#define PG_PID_CONFIG 504;

// Declare system config
#define PG_DECLARE(_type, _name)                                        \
    extern _type _name ## _System;                                      \
    static inline const _type* _name(void) { return &_name ## _System; }\
    static inline _type* _name ## Mutable(void) { return &_name ## _System; }\
    struct _dummy                                                       \
    /**/

#define PG_DECLARE_ARRAY(_type, _size, _name)                           \
    extern _type _name ## _SystemArray[_size];                          \
    static inline const _type* _name(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type* _name ## Mutable(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type (* _name ## _array(void))[_size] { return &_name ## _SystemArray; } \
    struct _dummy                                                       \
    /**/

// Register system config
#define PG_REGISTER_I(_type, _name, _pgn, _version, _reset)

#define PG_REGISTER(_type, _name, _pgn, _version)                       \
    PG_REGISTER_I(_type, _name, _pgn, _version, 0)                      \
    /**/

#define PG_REGISTER_WITH_RESET_FN(_type, _name, _pgn, _version)         \
    PG_REGISTER_I(_type, _name, _pgn, _version, 0)                      \
    /**/

#define PG_REGISTER_WITH_RESET_TEMPLATE(_type, _name, _pgn, _version)   \
    PG_REGISTER_I(_type, _name, _pgn, _version, 0)                      \
    /**/

#define PG_RESET_TEMPLATE(_type, _name, ...)                            \
    _type _name ## _System = { __VA_ARGS__ };                           \
    /**/

#define PG_RESET_TEMPLATE_DEF(_type, _name)                              \
    _type _name ## _System;                                              \
    /**/

#define PG_RESET_TEMPLATE_ARRAY_DEF(_type, _size, _name)                  \
    _type _name ## _SystemArray[_size];                                    \
    /**/

/* PARAMETER GROUP END */

/* TIME START */
// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t;
// microseconds time
typedef uint32_t timeUs_t;
/* TIME END */

typedef enum rc_alias {
     ROLL = 0,
     PITCH,
     YAW,
     THROTTLE,
     AUX1,
     AUX2,
     AUX3,
     AUX4,
     AUX5,
     AUX6,
     AUX7,
     AUX8
} rc_alias_e;

/* SERIAL START */
#define SERIAL_PORT_COUNT 1

typedef enum {
    MODE_RX = 1 << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_e;

typedef enum {
    BF_SERIAL_NOT_INVERTED  = 0 << 0,
    BF_SERIAL_INVERTED      = 1 << 0,
    BF_SERIAL_STOPBITS_1    = 0 << 1,
    BF_SERIAL_STOPBITS_2    = 1 << 1,
    BF_SERIAL_PARITY_NO     = 0 << 2,
    BF_SERIAL_PARITY_EVEN   = 1 << 2,
    BF_SERIAL_UNIDIR        = 0 << 3,
    BF_SERIAL_BIDIR         = 1 << 3,

    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    BF_SERIAL_BIDIR_OD      = 0 << 4,
    BF_SERIAL_BIDIR_PP      = 1 << 4
} portOptions_e;

typedef void (*serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app

typedef struct serialPort_s {

    //const struct serialPortVTable *vTable;

    uint8_t identifier;
    portMode_e mode;
    portOptions_e options;

    uint32_t baudRate;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    serialReceiveCallbackPtr rxCallback;
    void * espfcDevice;
} serialPort_t;

typedef enum {
    PORTSHARING_UNUSED = 0,
    PORTSHARING_NOT_SHARED,
    PORTSHARING_SHARED
} portSharing_e;

typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0),  // 1
    FUNCTION_GPS                 = (1 << 1),  // 2
    FUNCTION_TELEMETRY_FRSKY     = (1 << 2),  // 4
    FUNCTION_TELEMETRY_HOTT      = (1 << 3),  // 8
    FUNCTION_TELEMETRY_LTM       = (1 << 4),  // 16
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
    FUNCTION_RX_SERIAL           = (1 << 6),  // 64
    FUNCTION_BLACKBOX            = (1 << 7),  // 128
    FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
    FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
    FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
    FUNCTION_TELEMETRY_IBUS      = (1 << 12), // 4096
    FUNCTION_VTX_TRAMP           = (1 << 13), // 8192
    FUNCTION_RCSPLIT             = (1 << 14), // 16384
} serialPortFunction_e;

typedef enum {
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_USART7,
    SERIAL_PORT_USART8,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_400000,
    BAUD_460800,
    BAUD_500000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000
} baudRate_e;

typedef struct serialPortConfig_s {
    uint16_t functionMask;
    serialPortIdentifier_e identifier;
    uint8_t msp_baudrateIndex;
    uint8_t gps_baudrateIndex;
    uint8_t blackbox_baudrateIndex;
    uint8_t telemetry_baudrateIndex; // not used for all telemetry systems, e.g. HoTT only works at 19200.
} serialPortConfig_t;

typedef struct serialConfig_s {
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
    uint16_t serial_update_rate_hz;
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
} serialConfig_t;

PG_DECLARE(serialConfig_t, serialConfig);

extern const uint32_t baudRates[];

void mspSerialAllocatePorts(void);
void mspSerialReleasePortIfAllocated(serialPort_t *serialPort);
serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction);
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudrate, portMode_e mode, portOptions_e options);
serialPort_t *getSerialPort();
void closeSerialPort(serialPort_t *serialPort);
uint32_t serialRxBytesWaiting(serialPort_t * instance);
uint32_t serialTxBytesFree(const serialPort_t *instance);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function);

void serialDeviceInit(void * serial, size_t index);

void serialBeginWrite(serialPort_t * instance);
void serialEndWrite(serialPort_t * instance);
void serialWrite(serialPort_t *instance, uint8_t ch);
int serialRead(serialPort_t *instance);
/* SERIAL END */

/* MIXER START */
typedef enum mixerMode
{
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
    MIXER_QUADX_1234 = 26
} mixerMode_e;

typedef struct mixerConfig_s {
    uint8_t mixerMode;
    bool yaw_motors_reversed;
    uint8_t mixer_type;
} mixerConfig_t;

extern const char * const lookupTableMixerType[];

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct motorDevConfig_s {
    uint16_t motorPwmRate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;              // Pwm Protocol
    uint8_t  motorPwmInversion;             // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
    uint8_t  useUnsyncedPwm;
    uint8_t  useDshotTelemetry;
//    ioTag_t  ioTags[MAX_SUPPORTED_MOTORS];
} motorDevConfig_t;

typedef struct motorConfig_s {
    motorDevConfig_t dev;
    uint16_t digitalIdleOffsetValue;        // Idle value for DShot protocol, full motor output = 10000
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint8_t motorPoleCount;
} motorConfig_t;

PG_DECLARE(motorConfig_t, motorConfig);

#define RPM_FILTER_HARMONICS_MAX 3

typedef struct rpmFilterConfig_s {
    uint8_t  rpm_filter_harmonics;     // how many harmonics should be covered with notches? 0 means filter off
    uint8_t  rpm_filter_weights[RPM_FILTER_HARMONICS_MAX];  // effect or "weight" (0% - 100%) of each RPM filter harmonic
    uint8_t  rpm_filter_min_hz;        // minimum frequency of the notches
    uint16_t rpm_filter_fade_range_hz; // range in which to gradually turn off notches down to minHz
    uint16_t rpm_filter_q;             // q of the notches
    uint16_t rpm_filter_lpf_hz;        // the cutoff of the lpf on reported motor rpm
} rpmFilterConfig_t;

PG_DECLARE(rpmFilterConfig_t, rpmFilterConfig);

extern float motor[MAX_SUPPORTED_MOTORS];
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];

uint8_t getMotorCount();
bool areMotorsRunning(void);
/* MIXER END */

/* RCMODES START */
typedef enum {
    // ARM flag
    BOXARM = 0,
    // FLIGHT_MODE
    BOXANGLE,
    BOXHORIZON,
    BOXMAG,
    BOXHEADFREE,
    BOXPASSTHRU,
    BOXFAILSAFE,
    BOXGPSRESCUE,
    BOXID_FLIGHTMODE_LAST = BOXGPSRESCUE,
    // RCMODE flags
    BOXANTIGRAVITY,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXBEEPERON,
    BOXLEDLOW,
    BOXCALIB,
    BOXOSD,
    BOXTELEMETRY,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXBLACKBOX,
    BOXAIRMODE,
    BOX3D,
    BOXFPVANGLEMIX,
    BOXBLACKBOXERASE,
    BOXCAMERA1,
    BOXCAMERA2,
    BOXCAMERA3,
    BOXFLIPOVERAFTERCRASH,
    BOXPREARM,
    BOXBEEPGPSCOUNT,
    BOXVTXPITMODE,
    BOXPARALYZE,
    BOXUSER1,
    BOXUSER2,
    BOXUSER3,
    BOXUSER4,
    BOXPIDAUDIO,
    BOXACROTRAINER,
    BOXVTXCONTROLDISABLE,
    BOXLAUNCHCONTROL,
    CHECKBOX_ITEM_COUNT
} boxId_e;

typedef struct boxBitmask_s {
  uint32_t bits[(CHECKBOX_ITEM_COUNT + 31) / 32];
} boxBitmask_t;

#define BITARRAY_BIT_OP(array, bit, op) ((array)[(bit) / (sizeof((array)[0]) * 8)] op (1 << ((bit) % (sizeof((array)[0]) * 8))))

bool bitArrayGet(const void *array, unsigned bit);
void bitArraySet(void *array, unsigned bit);
void bitArrayClr(void *array, unsigned bit);

bool IS_RC_MODE_ACTIVE(boxId_e boxId);

typedef enum {
    ARMED                       = (1 << 0),
    WAS_EVER_ARMED              = (1 << 1),
    WAS_ARMED_WITH_PREARM       = (1 << 2)
} armingFlag_e;

extern uint8_t stateFlags;
extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

typedef enum {
    FEATURE_RX_PPM = 1 << 0,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_RX_SERIAL = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_SONAR = 1 << 9,
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_OSD = 1 << 18,
    FEATURE_CHANNEL_FORWARDING = 1 << 20,
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_AIRMODE = 1 << 22,
    FEATURE_RX_SPI = 1 << 25,
    FEATURE_SOFTSPI = 1 << 26,
    FEATURE_ESC_SENSOR = 1 << 27,
    FEATURE_ANTI_GRAVITY = 1 << 28,
    FEATURE_DYNAMIC_FILTER = 1 << 29,
} features_e;

bool featureIsEnabled(uint32_t mask);

typedef struct featureConfig_s {
    uint32_t enabledFeatures;
} featureConfig_t;

PG_DECLARE(featureConfig_t, featureConfig);

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
} flight3DConfig_t;

PG_DECLARE(flight3DConfig_t, flight3DConfig);

typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_deadband;              // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
    uint8_t alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
    bool yaw_control_reversed;            // invert control direction of yaw
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

extern uint16_t rssi;
extern boxBitmask_t rcModeActivationMask;
extern boxBitmask_t rcModeActivationPresent;

bool isModeActivationConditionPresent(boxId_e modeId);
uint32_t getArmingBeepTimeMicros(void);
void setArmingBeepTimeMicros(uint32_t ts);
/* RCMODES END */

/* CONFIG START */
#define MAX_NAME_LENGTH 16u
typedef struct pilotConfig_s {
    char craftName[MAX_NAME_LENGTH + 1];
} pilotConfig_t;

PG_DECLARE(pilotConfig_t, pilotConfig);

typedef struct systemConfig_s {
    uint8_t pidProfileIndex;
    uint8_t activeRateProfile;
    uint8_t debug_mode;
} systemConfig_t;

PG_DECLARE(systemConfig_t, systemConfig);

typedef struct controlRateConfig_s {
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates_type;
    uint8_t rcRates[3];
    uint8_t rcExpo[3];
    uint8_t rates[3];
    uint16_t rate_limit[3];
    uint8_t dynThrPID;
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint8_t throttle_limit_type;            // Sets the throttle limiting type - off, scale or clip
    uint8_t throttle_limit_percent;         // Sets the maximum pilot commanded throttle limit
} controlRateConfig_t;

#define CONTROL_RATE_PROFILE_COUNT  1
PG_DECLARE_ARRAY(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);

/* CONFIG END */

/* PID START */
#define MAX_PROFILE_COUNT 1

typedef enum {
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
} pidIndex_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
} pidf_t;

typedef struct pidProfile_s {
    pidf_t  pid[PID_ITEM_COUNT];
    uint16_t yaw_lowpass_hz;                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff
    uint8_t dterm_lpf1_type;                // Filter selection for dterm
    uint16_t dterm_lpf1_static_hz;          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t dterm_lpf1_dyn_min_hz;         // Dterm lowpass filter 1 min hz when in dynamic mode
    uint16_t dterm_lpf1_dyn_max_hz;         // Dterm lowpass filter 1 max hz when in dynamic mode
    uint8_t dterm_lpf1_dyn_expo;            // set the curve for dynamic dterm lowpass filter
    uint8_t itermWindupPointPercent;        // Experimental ITerm windup threshold, percent motor saturation
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t vbatPidCompensation;            // Scale PIDsum to battery voltage
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t levelAngleLimit;                // Max angle in degrees in level mode
    uint8_t levelSensitivity;               // Angle mode sensitivity reflected in degrees assuming user using full stick
    uint16_t itermThrottleThreshold;        // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain;          // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint8_t antiGravityMode;
    int16_t feedForwardTransition;
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t itermLimit;
    uint8_t ff_boost;
    uint8_t dterm_lpf2_type;                // Filter type for 2nd dterm lowpass
    uint16_t dterm_lpf2_static_hz;          // Static Dterm lowpass 2 filter cutoff value in hz
    uint8_t tpa_mode;                       // Controls which PID terms TPA effects
    uint8_t tpa_rate;                       // Percent reduction in P or D at full throttle
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint8_t motor_output_limit;             // Upper limit of the motor output (percent)
    uint8_t throttle_boost;                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    uint8_t throttle_boost_cutoff;          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    uint8_t anti_gravity_gain;              // AntiGravity Gain (was itermAcceleratorGain)
    uint8_t anti_gravity_cutoff_hz;
    uint8_t anti_gravity_p_gain;
    uint8_t d_min[XYZ_AXIS_COUNT];          // Minimum D value on each axis
    uint8_t d_min_gain;                     // Gain factor for amount of gyro / setpoint activity required to boost D
    uint8_t d_min_advance;                  // Percentage multiplier for setpoint input to boost algorithm
    uint8_t angle_limit;                    // Max angle in degrees in Angle mode
    uint8_t angle_earth_ref;                // Control amount of "co-ordination" from yaw into roll while pitched forward in angle mode
    uint8_t horizon_limit_degrees;          // in Horizon mode, zero levelling when the quad's attitude exceeds this angle
    uint16_t horizon_delay_ms;              // delay when Horizon Strength increases, 50 = 500ms time constant
    uint8_t thrustLinearization;            // Compensation factor for pid linearization
    uint8_t iterm_relax_type;               // Specifies type of relax algorithm
    uint8_t iterm_relax_cutoff;             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax;                    // Enable iterm suppression during stick input
    uint8_t tpa_low_rate;                   // Percent reduction in P or D at zero throttle
    uint16_t tpa_low_breakpoint;            // Breakpoint where lower TPA is deactivated
    uint8_t tpa_low_always;                 // off, on - if OFF then low TPA is only active until tpa_low_breakpoint is reached the first time
    uint8_t ez_landing_threshold;           // Threshold stick position below which motor output is limited
    uint8_t ez_landing_limit;               // Maximum motor output when all sticks centred and throttle zero
} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

extern pidAxisData_t pidData[3];

extern struct pidProfile_s *currentPidProfile;
extern uint32_t targetPidLooptime;
/* PID END */

/* DEBUG START */
typedef enum {
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
} debugType_e;

extern float rcCommand[4];
extern uint8_t debugMode;
extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t activePidLoopDenom;
/* DEBUG END */

/* SENSOR START */
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);

typedef enum {
    SENSOR_GYRO   = 1 << 0, // always present
    SENSOR_ACC    = 1 << 1,
    SENSOR_BARO   = 1 << 2,
    SENSOR_MAG    = 1 << 3,
    SENSOR_SONAR  = 1 << 4,
    SENSOR_GPS    = 1 << 5,
    SENSOR_GPSMAG = 1 << 6
} sensors_e;


typedef struct gyro_s {
    //uint32_t targetLooptime;
    uint32_t sampleLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];
} gyro_t;
extern gyro_t gyro;

typedef struct accDev_s {
    uint16_t acc_1G;
} accDev_t;

typedef struct acc_s {
    accDev_t dev;
    int32_t accADC[XYZ_AXIS_COUNT];
} acc_t;
extern acc_t acc;

typedef struct mag_s {
    float magADC[XYZ_AXIS_COUNT];
} mag_t;
extern mag_t mag;

typedef struct baro_s {
   int32_t altitude;
} baro_t;
extern baro_t baro;

typedef enum {
    VOLTAGE_METER_NONE = 0,
    VOLTAGE_METER_ADC,
    VOLTAGE_METER_ESC
} voltageMeterSource_e;

// WARNING - do not mix usage of VOLTAGE_METER_* and VOLTAGE_SENSOR_*, they are separate concerns.

typedef struct voltageMeter_s {
    uint16_t filtered;                      // voltage in 0.1V steps
    uint16_t unfiltered;                    // voltage in 0.1V steps
    bool lowVoltageCutoff;
} voltageMeter_t;

typedef enum {
    CURRENT_METER_NONE = 0,
    CURRENT_METER_ADC,
    CURRENT_METER_VIRTUAL,
    CURRENT_METER_ESC,
    CURRENT_METER_MSP,
    CURRENT_METER_MAX = CURRENT_METER_ESC
} currentMeterSource_e;

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_NONE
#define DEFAULT_CURRENT_METER_SOURCE VOLTAGE_METER_NONE

typedef struct batteryConfig_s {
    // voltage
    uint16_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint16_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    uint16_t vbatwarningcellvoltage;         // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    uint8_t vbatnotpresentcellvoltage;      // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    voltageMeterSource_e voltageMeterSource; // source of battery voltage meter used, either ADC or ESC
    currentMeterSource_e currentMeterSource; // source of battery current meter used, either ADC, Virtual or ESC
} batteryConfig_t;

PG_DECLARE(batteryConfig_t, batteryConfig);

#define MAX_VOLTAGE_SENSOR_ADC 1
#define VOLTAGE_SENSOR_ADC_VBAT 0

typedef struct voltageSensorADCConfig_s {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatresdivval;                  // resistor divider R2 (default NAZE 10(K))
    uint8_t vbatresdivmultiplier;           // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
} voltageSensorADCConfig_t;

PG_DECLARE_ARRAY(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);

typedef struct compassConfig_s {
    int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
} compassConfig_t;

PG_DECLARE(compassConfig_t, compassConfig);

typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
} accelerometerConfig_t;

PG_DECLARE(accelerometerConfig_t, accelerometerConfig);

typedef struct barometerConfig_s {
    uint8_t baro_hardware;                  // Barometer hardware to use
} barometerConfig_t;

PG_DECLARE(barometerConfig_t, barometerConfig);

typedef struct positionConfig_s {
    uint8_t altitude_source;
    uint8_t altitude_prefer_baro;
    uint16_t altitude_lpf;                // lowpass cutoff (value / 100) Hz for altitude smoothing
    uint16_t altitude_d_lpf;              // lowpass for (value / 100) Hz for altitude derivative smoothing
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

typedef struct gyroConfig_s {
    //sensor_align_e gyro_align;              // gyro alignment
    //uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_sync_denom;                  // Gyro sample divider
    uint8_t  gyro_hardware_lpf;                         // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t  gyro_to_use;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    uint8_t gyro_lpf1_type;
    uint16_t gyro_lpf1_static_hz;
    uint16_t gyro_lpf1_dyn_min_hz;
    uint16_t gyro_lpf1_dyn_max_hz;
    uint8_t gyro_lpf1_dyn_expo; // set the curve for dynamic gyro lowpass filter
    uint8_t gyro_lpf2_type;
    uint16_t gyro_lpf2_static_hz;
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

typedef struct dynNotchConfig_s
{
    uint16_t dyn_notch_min_hz;
    uint16_t dyn_notch_max_hz;
    uint16_t dyn_notch_q;
    uint8_t  dyn_notch_count;

} dynNotchConfig_t;

PG_DECLARE(dynNotchConfig_t, dynNotchConfig);

typedef struct currentSensorADCConfig_s {
    int16_t scale;              // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    int16_t offset;            // offset of the current sensor in millivolt steps
} currentSensorADCConfig_t;

PG_DECLARE(currentSensorADCConfig_t, currentSensorADCConfig);

uint16_t getBatteryVoltageLatest(void);
int32_t getAmperageLatest(void);
/* SENSOR END */

/* RX START */
#define RX_MAPPABLE_CHANNEL_COUNT 8

typedef struct rxConfig_s {
    uint8_t serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t rssi_channel;
    uint8_t rcInterpolation;
    uint8_t rcInterpolationChannels;
    uint8_t rcInterpolationInterval;
    uint16_t airModeActivateThreshold;      // Throttle setpoint where airmode gets activated
} rxConfig_t;

PG_DECLARE(rxConfig_t, rxConfig);

uint16_t getRssi(void);

/* FAILSAFE START */
typedef enum {
    FAILSAFE_IDLE = 0,
    FAILSAFE_RX_LOSS_DETECTED,
    FAILSAFE_LANDING,
    FAILSAFE_LANDED,
    FAILSAFE_RX_LOSS_MONITORING,
    FAILSAFE_RX_LOSS_RECOVERED
} failsafePhase_e;

failsafePhase_e failsafePhase();
bool rxIsReceivingSignal(void);
bool rxAreFlightChannelsValid(void);
float pidGetPreviousSetpoint(int axis);
float mixerGetThrottle(void);
bool isRssiConfigured(void);
int16_t getMotorOutputLow();
int16_t getMotorOutputHigh();
uint16_t getDshotErpm(uint8_t i);
/* FAILSAFE END */

#define PARAM_NAME_GYRO_HARDWARE_LPF "gyro_hardware_lpf"
#define PARAM_NAME_GYRO_LPF1_TYPE "gyro_lpf1_type"
#define PARAM_NAME_GYRO_LPF1_STATIC_HZ "gyro_lpf1_static_hz"
#define PARAM_NAME_GYRO_LPF2_TYPE "gyro_lpf2_type"
#define PARAM_NAME_GYRO_LPF2_STATIC_HZ "gyro_lpf2_static_hz"
#define PARAM_NAME_GYRO_TO_USE "gyro_to_use"
#define PARAM_NAME_DYN_NOTCH_MAX_HZ "dyn_notch_max_hz"
#define PARAM_NAME_DYN_NOTCH_COUNT "dyn_notch_count"
#define PARAM_NAME_DYN_NOTCH_Q "dyn_notch_q"
#define PARAM_NAME_DYN_NOTCH_MIN_HZ "dyn_notch_min_hz"
#define PARAM_NAME_ACC_HARDWARE "acc_hardware"
#define PARAM_NAME_ACC_LPF_HZ "acc_lpf_hz"
#define PARAM_NAME_MAG_HARDWARE "mag_hardware"
#define PARAM_NAME_BARO_HARDWARE "baro_hardware"
#define PARAM_NAME_RC_SMOOTHING "rc_smoothing"
#define PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR "rc_smoothing_auto_factor"
#define PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR_THROTTLE "rc_smoothing_auto_factor_throttle"
#define PARAM_NAME_RC_SMOOTHING_SETPOINT_CUTOFF "rc_smoothing_setpoint_cutoff"
#define PARAM_NAME_RC_SMOOTHING_FEEDFORWARD_CUTOFF "rc_smoothing_feedforward_cutoff"
#define PARAM_NAME_RC_SMOOTHING_THROTTLE_CUTOFF "rc_smoothing_throttle_cutoff"
#define PARAM_NAME_RC_SMOOTHING_DEBUG_AXIS "rc_smoothing_debug_axis"
#define PARAM_NAME_RC_SMOOTHING_ACTIVE_CUTOFFS "rc_smoothing_active_cutoffs_ff_sp_thr"
#define PARAM_NAME_SERIAL_RX_PROVIDER "serialrx_provider"
#define PARAM_NAME_DSHOT_IDLE_VALUE "dshot_idle_value"
#define PARAM_NAME_DSHOT_BIDIR "dshot_bidir"
#define PARAM_NAME_USE_UNSYNCED_PWM "use_unsynced_pwm"
#define PARAM_NAME_MOTOR_PWM_PROTOCOL "motor_pwm_protocol"
#define PARAM_NAME_MOTOR_PWM_RATE "motor_pwm_rate"
#define PARAM_NAME_MOTOR_POLES "motor_poles"
#define PARAM_NAME_THR_MID "thr_mid"
#define PARAM_NAME_THR_EXPO "thr_expo"
#define PARAM_NAME_RATES_TYPE "rates_type"
#define PARAM_NAME_TPA_RATE "tpa_rate"
#define PARAM_NAME_TPA_BREAKPOINT "tpa_breakpoint"
#define PARAM_NAME_TPA_LOW_RATE "tpa_low_rate"
#define PARAM_NAME_TPA_LOW_BREAKPOINT "tpa_low_breakpoint"
#define PARAM_NAME_TPA_LOW_ALWAYS "tpa_low_always"
#define PARAM_NAME_TPA_MODE "tpa_mode"
#define PARAM_NAME_MIXER_TYPE "mixer_type"
#define PARAM_NAME_EZ_LANDING_THRESHOLD "ez_landing_threshold"
#define PARAM_NAME_EZ_LANDING_LIMIT "ez_landing_limit"
#define PARAM_NAME_THROTTLE_LIMIT_TYPE "throttle_limit_type"
#define PARAM_NAME_THROTTLE_LIMIT_PERCENT "throttle_limit_percent"
#define PARAM_NAME_GYRO_CAL_ON_FIRST_ARM "gyro_cal_on_first_arm"
#define PARAM_NAME_DEADBAND "deadband"
#define PARAM_NAME_YAW_DEADBAND "yaw_deadband"
#define PARAM_NAME_PID_PROCESS_DENOM "pid_process_denom"
#define PARAM_NAME_DTERM_LPF1_TYPE "dterm_lpf1_type"
#define PARAM_NAME_DTERM_LPF1_STATIC_HZ "dterm_lpf1_static_hz"
#define PARAM_NAME_DTERM_LPF2_TYPE "dterm_lpf2_type"
#define PARAM_NAME_DTERM_LPF2_STATIC_HZ "dterm_lpf2_static_hz"
#define PARAM_NAME_DTERM_NOTCH_HZ "dterm_notch_hz"
#define PARAM_NAME_DTERM_NOTCH_CUTOFF "dterm_notch_cutoff"
#define PARAM_NAME_VBAT_SAG_COMPENSATION "vbat_sag_compensation"
#define PARAM_NAME_PID_AT_MIN_THROTTLE "pid_at_min_throttle"
#define PARAM_NAME_ANTI_GRAVITY_GAIN "anti_gravity_gain"
#define PARAM_NAME_ANTI_GRAVITY_CUTOFF_HZ "anti_gravity_cutoff_hz"
#define PARAM_NAME_ANTI_GRAVITY_P_GAIN "anti_gravity_p_gain"
#define PARAM_NAME_ACC_LIMIT_YAW "acc_limit_yaw"
#define PARAM_NAME_ACC_LIMIT "acc_limit"
#define PARAM_NAME_ITERM_RELAX "iterm_relax"
#define PARAM_NAME_ITERM_RELAX_TYPE "iterm_relax_type"
#define PARAM_NAME_ITERM_RELAX_CUTOFF "iterm_relax_cutoff"
#define PARAM_NAME_ITERM_WINDUP "iterm_windup"
#define PARAM_NAME_PIDSUM_LIMIT "pidsum_limit"
#define PARAM_NAME_PIDSUM_LIMIT_YAW "pidsum_limit_yaw"
#define PARAM_NAME_YAW_LOWPASS_HZ "yaw_lowpass_hz"
#define PARAM_NAME_THROTTLE_BOOST "throttle_boost"
#define PARAM_NAME_THROTTLE_BOOST_CUTOFF "throttle_boost_cutoff"
#define PARAM_NAME_THRUST_LINEARIZATION "thrust_linear"
#define PARAM_NAME_ABS_CONTROL_GAIN "abs_control_gain"
#define PARAM_NAME_USE_INTEGRATED_YAW "use_integrated_yaw"
#define PARAM_NAME_D_MAX_GAIN "d_max_gain"
#define PARAM_NAME_D_MAX_ADVANCE "d_max_advance"
#define PARAM_NAME_MOTOR_OUTPUT_LIMIT "motor_output_limit"
#define PARAM_NAME_FEEDFORWARD_TRANSITION "feedforward_transition"
#define PARAM_NAME_FEEDFORWARD_AVERAGING "feedforward_averaging"
#define PARAM_NAME_FEEDFORWARD_SMOOTH_FACTOR "feedforward_smooth_factor"
#define PARAM_NAME_FEEDFORWARD_JITTER_FACTOR "feedforward_jitter_factor"
#define PARAM_NAME_FEEDFORWARD_BOOST "feedforward_boost"
#define PARAM_NAME_FEEDFORWARD_MAX_RATE_LIMIT "feedforward_max_rate_limit"
#define PARAM_NAME_DYN_IDLE_MIN_RPM "dyn_idle_min_rpm"
#define PARAM_NAME_DYN_IDLE_P_GAIN "dyn_idle_p_gain"
#define PARAM_NAME_DYN_IDLE_I_GAIN "dyn_idle_i_gain"
#define PARAM_NAME_DYN_IDLE_D_GAIN "dyn_idle_d_gain"
#define PARAM_NAME_DYN_IDLE_MAX_INCREASE "dyn_idle_max_increase"
#define PARAM_NAME_DYN_IDLE_START_INCREASE "dyn_idle_start_increase"
#define PARAM_NAME_SIMPLIFIED_PIDS_MODE "simplified_pids_mode"
#define PARAM_NAME_SIMPLIFIED_MASTER_MULTIPLIER "simplified_master_multiplier"
#define PARAM_NAME_SIMPLIFIED_I_GAIN "simplified_i_gain"
#define PARAM_NAME_SIMPLIFIED_D_GAIN "simplified_d_gain"
#define PARAM_NAME_SIMPLIFIED_PI_GAIN "simplified_pi_gain"
#define PARAM_NAME_SIMPLIFIED_DMAX_GAIN "simplified_dmax_gain"
#define PARAM_NAME_SIMPLIFIED_FEEDFORWARD_GAIN "simplified_feedforward_gain"
#define PARAM_NAME_SIMPLIFIED_PITCH_D_GAIN "simplified_pitch_d_gain"
#define PARAM_NAME_SIMPLIFIED_PITCH_PI_GAIN "simplified_pitch_pi_gain"
#define PARAM_NAME_SIMPLIFIED_DTERM_FILTER "simplified_dterm_filter"
#define PARAM_NAME_SIMPLIFIED_DTERM_FILTER_MULTIPLIER "simplified_dterm_filter_multiplier"
#define PARAM_NAME_SIMPLIFIED_GYRO_FILTER "simplified_gyro_filter"
#define PARAM_NAME_SIMPLIFIED_GYRO_FILTER_MULTIPLIER "simplified_gyro_filter_multiplier"
#define PARAM_NAME_DEBUG_MODE "debug_mode"
#define PARAM_NAME_RPM_FILTER_HARMONICS "rpm_filter_harmonics"
#define PARAM_NAME_RPM_FILTER_WEIGHTS "rpm_filter_weights"
#define PARAM_NAME_RPM_FILTER_Q "rpm_filter_q"
#define PARAM_NAME_RPM_FILTER_MIN_HZ "rpm_filter_min_hz"
#define PARAM_NAME_RPM_FILTER_FADE_RANGE_HZ "rpm_filter_fade_range_hz"
#define PARAM_NAME_RPM_FILTER_LPF_HZ "rpm_filter_lpf_hz"
#define PARAM_NAME_POSITION_ALTITUDE_SOURCE "altitude_source"
#define PARAM_NAME_POSITION_ALTITUDE_PREFER_BARO "altitude_prefer_baro"
#define PARAM_NAME_POSITION_ALTITUDE_LPF "altitude_lpf"
#define PARAM_NAME_POSITION_ALTITUDE_D_LPF "altitude_d_lpf"
#define PARAM_NAME_ANGLE_FEEDFORWARD "angle_feedforward"
#define PARAM_NAME_ANGLE_FF_SMOOTHING_MS "angle_feedforward_smoothing_ms"
#define PARAM_NAME_ANGLE_LIMIT "angle_limit"
#define PARAM_NAME_ANGLE_P_GAIN "angle_p_gain"
#define PARAM_NAME_ANGLE_EARTH_REF "angle_earth_ref"

#define PARAM_NAME_HORIZON_LEVEL_STRENGTH "horizon_level_strength"
#define PARAM_NAME_HORIZON_LIMIT_DEGREES "horizon_limit_degrees"
#define PARAM_NAME_HORIZON_LIMIT_STICKS "horizon_limit_sticks"
#define PARAM_NAME_HORIZON_IGNORE_STICKS "horizon_ignore_sticks"
#define PARAM_NAME_HORIZON_DELAY_MS "horizon_delay_ms"

#ifdef USE_GPS
#define PARAM_NAME_GPS_PROVIDER "gps_provider"
#define PARAM_NAME_GPS_SBAS_MODE "gps_sbas_mode"
#define PARAM_NAME_GPS_SBAS_INTEGRITY "gps_sbas_integrity"
#define PARAM_NAME_GPS_AUTO_CONFIG "gps_auto_config"
#define PARAM_NAME_GPS_AUTO_BAUD "gps_auto_baud"
#define PARAM_NAME_GPS_UBLOX_USE_GALILEO "gps_ublox_use_galileo"
#define PARAM_NAME_GPS_UBLOX_FULL_POWER "gps_ublox_full_power"
#define PARAM_NAME_GPS_UBLOX_ACQUIRE_MODEL "gps_ublox_acquire_model"
#define PARAM_NAME_GPS_UBLOX_FLIGHT_MODEL "gps_ublox_flight_model"
#define PARAM_NAME_GPS_SET_HOME_POINT_ONCE "gps_set_home_point_once"
#define PARAM_NAME_GPS_USE_3D_SPEED "gps_use_3d_speed"
#define PARAM_NAME_GPS_NMEA_CUSTOM_COMMANDS "gps_nmea_custom_commands"
#define PARAM_NAME_GPS_UPDATE_RATE_HZ "gps_update_rate_hz"

#ifdef USE_GPS_RESCUE
#define PARAM_NAME_GPS_RESCUE_MIN_START_DIST "gps_rescue_min_start_dist"
#define PARAM_NAME_GPS_RESCUE_ALT_MODE "gps_rescue_alt_mode"
#define PARAM_NAME_GPS_RESCUE_INITIAL_CLIMB "gps_rescue_initial_climb"
#define PARAM_NAME_GPS_RESCUE_ASCEND_RATE "gps_rescue_ascend_rate"

#define PARAM_NAME_GPS_RESCUE_RETURN_ALT "gps_rescue_return_alt"
#define PARAM_NAME_GPS_RESCUE_RETURN_SPEED "gps_rescue_ground_speed"
#define PARAM_NAME_GPS_RESCUE_MAX_RESCUE_ANGLE "gps_rescue_max_angle"
#define PARAM_NAME_GPS_RESCUE_ROLL_MIX "gps_rescue_roll_mix"
#define PARAM_NAME_GPS_RESCUE_PITCH_CUTOFF "gps_rescue_pitch_cutoff"

#define PARAM_NAME_GPS_RESCUE_DESCENT_DIST "gps_rescue_descent_dist"
#define PARAM_NAME_GPS_RESCUE_DESCEND_RATE "gps_rescue_descend_rate"
#define PARAM_NAME_GPS_RESCUE_LANDING_ALT "gps_rescue_landing_alt"
#define PARAM_NAME_GPS_RESCUE_DISARM_THRESHOLD "gps_rescue_disarm_threshold"

#define PARAM_NAME_GPS_RESCUE_THROTTLE_MIN "gps_rescue_throttle_min"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_MAX "gps_rescue_throttle_max"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_HOVER "gps_rescue_throttle_hover"

#define PARAM_NAME_GPS_RESCUE_SANITY_CHECKS "gps_rescue_sanity_checks"
#define PARAM_NAME_GPS_RESCUE_MIN_SATS "gps_rescue_min_sats"
#define PARAM_NAME_GPS_RESCUE_ALLOW_ARMING_WITHOUT_FIX "gps_rescue_allow_arming_without_fix"

#define PARAM_NAME_GPS_RESCUE_THROTTLE_P "gps_rescue_throttle_p"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_I "gps_rescue_throttle_i"
#define PARAM_NAME_GPS_RESCUE_THROTTLE_D "gps_rescue_throttle_d"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_P "gps_rescue_velocity_p"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_I "gps_rescue_velocity_i"
#define PARAM_NAME_GPS_RESCUE_VELOCITY_D "gps_rescue_velocity_d"
#define PARAM_NAME_GPS_RESCUE_YAW_P "gps_rescue_yaw_p"

#ifdef USE_MAG
#define PARAM_NAME_GPS_RESCUE_USE_MAG "gps_rescue_use_mag"
#endif
#endif

#ifdef USE_GPS_LAP_TIMER
#define PARAM_NAME_GPS_LAP_TIMER_GATE_LAT "gps_lap_timer_gate_lat"
#define PARAM_NAME_GPS_LAP_TIMER_GATE_LON "gps_lap_timer_gate_lon"
#define PARAM_NAME_GPS_LAP_TIMER_MIN_LAP_TIME "gps_lap_timer_min_lap_time_s"
#define PARAM_NAME_GPS_LAP_TIMER_GATE_TOLERANCE "gps_lap_timer_gate_tolerance_m"
#endif // USE_GPS_LAP_TIMER
#endif

// ESC 4-Way IF

#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_SERIAL_4WAY_SK_BOOTLOADER

typedef int8_t IO_t;

typedef struct {
    bool enabled;
    IO_t io;
} pwmOutputPort_t;

pwmOutputPort_t * pwmGetMotors(void);
void motorDisable(void);
void motorEnable(void);
void motorInitEscDevice(void * driver);

#if defined(UNIT_TEST) || defined(ESP8266) || defined(ARCH_RP2040)
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
#else
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
#endif

#define IOCFG_IPU    0x00 // INPUT_PULLUP
#define IOCFG_OUT_PP 0x01 // OUTPUT
#define IOCFG_AF_PP  0x02 // OUTPUT
#define Bit_RESET    0x00 // LOW
#define IO_NONE      -1

int IORead(IO_t pin);
void IOConfigGPIO(IO_t pin, uint8_t mode);
void IOHi(IO_t pin);
void IOLo(IO_t pin);

#define LED0_ON do {} while(false)
#define LED0_OFF do {} while(false)

// end ESC 4-Way IF

typedef enum {
    // IMPORTANT: the order of the elements should be preserved for backwards compatibility with the configurator.
    BEEPER_SILENCE = 0,             // Silence, see beeperSilence()

    BEEPER_GYRO_CALIBRATED,
    BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
    BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
    BEEPER_DISARMING,               // Beep when disarming the board
    BEEPER_ARMING,                  // Beep when arming the board
    BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
    BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
    BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
    BEEPER_GPS_STATUS,              // Use the number of beeps to indicate how many GPS satellites were found
    BEEPER_RX_SET,                  // Beeps when aux channel is set for beep
    BEEPER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
    BEEPER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
    BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
    BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
    BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
    BEEPER_ARMED,                   // Warning beeps when board is armed with motors off when idle (repeats until board is disarmed or throttle is increased)
    BEEPER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
    BEEPER_USB,                     // Some boards have beeper powered USB connected
    BEEPER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
    BEEPER_CRASH_FLIP_MODE,         // Crash flip mode is active
    BEEPER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
    BEEPER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
    BEEPER_RC_SMOOTHING_INIT_FAIL,  // Warning beep pattern when armed and rc smoothing has not initialized filters
    BEEPER_ARMING_GPS_NO_FIX,       // Beep a special tone when arming the board and GPS has no fix
    BEEPER_ALL,                     // Turn ON or OFF all beeper conditions
    // BEEPER_ALL must remain at the bottom of this enum
} beeperMode_e;

void beeper(int mode);

// FLASHFS START
#ifdef USE_FLASHFS

#define FLASHFS_WRITE_BUFFER_SIZE 128u
#define FLASHFS_FLUSH_BUFFER_SIZE 64u
#define FLASHFS_JOURNAL_ITEMS 32u

typedef struct
{
    uint32_t logBegin;
    uint32_t logEnd;
} __attribute__((packed)) FlashfsJournalItem;

typedef struct
{
    const void* partition;
    void* buffer;
    uint32_t address;
    FlashfsJournalItem journal[FLASHFS_JOURNAL_ITEMS];
    uint32_t journalIdx;
} FlashfsRuntime;

#define FLASHFS_JOURNAL_SIZE (FLASHFS_JOURNAL_ITEMS * sizeof(FlashfsJournalItem))

const FlashfsRuntime * flashfsGetRuntime();

void flashfsJournalLoad(FlashfsJournalItem * data, size_t start, size_t num);

int flashfsInit(void);
uint32_t flashfsGetSize(void);
uint32_t flashfsGetOffset(void);
uint32_t flashfsGetSectors(void);

void flashfsWriteByte(uint8_t byte);
void flashfsWrite(const uint8_t *data, unsigned int len, bool sync);

void flashfsWriteAbs(uint32_t address, const uint8_t *data, unsigned int len);
int flashfsReadAbs(uint32_t address, uint8_t *data, unsigned int len);

bool flashfsFlushAsync(bool force);

bool flashfsIsSupported(void);
bool flashfsIsReady(void);
bool flashfsIsEOF(void);

void flashfsEraseCompletely(void);
void flashfsClose(void);

uint32_t flashfsGetWriteBufferFreeSpace(void);
uint32_t flashfsGetWriteBufferSize(void);

#endif

#ifdef __cplusplus
}
#endif
