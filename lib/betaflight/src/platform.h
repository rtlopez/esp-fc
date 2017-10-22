#pragma once

#define BLACKBOX

#include <stdbool.h>
#include <stdint.h>
#include "printf.h"

#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 0
#define PID_PROCESS_DENOM_DEFAULT       1

#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_VERSION_MAJOR            3  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            2  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

#define MW_VERSION                1

extern const char * const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char * const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char * const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char * const buildTime;  // "HH:MM:SS"

#define TARGET_BOARD_IDENTIFIER "ESPFC"

/* UTILS START */
#define MIN(a,b) ((a > b) ? (b) : (a))
#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)
#define CONCAT2(_1,_2) CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)  CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)
#define XYZ_AXIS_COUNT 3
#define DEBUG16_VALUE_COUNT 4

#define STATIC_UNIT_TESTED static
#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)
#define UNUSED(v) ((void)v)
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define STATIC_ASSERT(condition, name) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ] __attribute__((unused))

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count);
uint32_t castFloatBytesToInt(float f);
uint32_t zigzagEncode(int32_t value);

int gcd(int num, int denom);

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

unsigned long millis(void);
unsigned long micros(void);
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
//#define PG_REGISTER_I(_type, _name, _pgn, _version, _reset)             \
//    _type _name ## _System;                                             \
//    /**/

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
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3,

    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    SERIAL_BIDIR_OD      = 0 << 4,
    SERIAL_BIDIR_PP      = 1 << 4
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

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction);
void mspSerialReleasePortIfAllocated(struct serialPort_s *serialPort);
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_e mode, portOptions_e options);
void closeSerialPort(serialPort_t *serialPort);
void mspSerialAllocatePorts(void);
uint32_t serialTxBytesFree(const serialPort_t *instance);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function);

void serialWrite(serialPort_t *instance, uint8_t ch);
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
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct motorDevConfig_s {
    uint16_t motorPwmRate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;              // Pwm Protocol
    uint8_t  motorPwmInversion;             // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
    uint8_t  useUnsyncedPwm;
//    ioTag_t  ioTags[MAX_SUPPORTED_MOTORS];
} motorDevConfig_t;

typedef struct motorConfig_s {
    motorDevConfig_t dev;
    uint16_t digitalIdleOffsetValue;        // Idle value for DShot protocol, full motor output = 10000
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
} motorConfig_t;

PG_DECLARE(motorConfig_t, motorConfig);

extern float motor[MAX_SUPPORTED_MOTORS];
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];
extern float motorOutputHigh, motorOutputLow;

uint8_t getMotorCount();
bool areMotorsRunning(void);
/* MIXER END */

/* RCMODES START */
typedef enum {
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXBARO,
    BOXANTIGRAVITY,
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLEDLOW,
    BOXLLIGHTS,
    BOXCALIB,
    BOXGOV,
    BOXOSD,
    BOXTELEMETRY,
    BOXGTUNE,
    BOXSONAR,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXBLACKBOX,
    BOXFAILSAFE,
    BOXAIRMODE,
    BOX3DDISABLE,
    BOXFPVANGLEMIX,
    BOXBLACKBOXERASE,
    BOXCAMERA1,
    BOXCAMERA2,
    BOXCAMERA3,
    BOXDSHOTREVERSE,
    BOXPREARM,
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

extern uint16_t flightModeFlags;
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

bool feature(uint32_t mask);

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

bool isModeActivationConditionPresent(boxId_e modeId);
uint32_t getArmingBeepTimeMicros(void);
void setArmingBeepTimeMicros(uint32_t ts);
/* RCMODES END */

/* CONFIG START */
#define MAX_NAME_LENGTH 16u
typedef struct pilotConfig_s {
    char name[MAX_NAME_LENGTH + 1];
} pilotConfig_t;

PG_DECLARE(pilotConfig_t, pilotConfig);

typedef struct systemConfig_s {
    uint8_t pidProfileIndex;
    uint8_t activeRateProfile;
    uint8_t debug_mode;
    uint8_t task_statistics;
#if defined(STM32F4) && !defined(DISABLE_OVERCLOCK)
    uint8_t cpu_overclock;
#endif
    uint8_t powerOnArmingGraceTime; // in seconds
    char boardIdentifier[sizeof(TARGET_BOARD_IDENTIFIER) + 1];
} systemConfig_t;

PG_DECLARE(systemConfig_t, systemConfig);

typedef struct controlRateConfig_s {
    uint8_t rcRate8;
    uint8_t rcYawRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates[3];
    uint8_t dynThrPID;
    uint8_t rcYawExpo8;
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
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

typedef struct pid8_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
} pid8_t;

typedef struct pidProfile_s {
    pid8_t  pid[PID_ITEM_COUNT];

    uint16_t yaw_lpf_hz;                    // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_lpf_hz;                  // Delta Filter in hz
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff
    uint8_t dterm_filter_type;              // Filter selection for dterm
    uint8_t itermWindupPointPercent;        // Experimental ITerm windup threshold, percent motor saturation
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t vbatPidCompensation;            // Scale PIDsum to battery voltage
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t levelAngleLimit;                // Max angle in degrees in level mode
    uint8_t levelSensitivity;               // Angle mode sensitivity reflected in degrees assuming user using full stick

    uint8_t horizon_tilt_effect;            // inclination factor for Horizon mode
    uint8_t horizon_tilt_expert_mode;       // OFF or ON

    // Betaflight PID controller parameters
    uint16_t itermThrottleThreshold;        // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain;          // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint8_t setpointRelaxRatio;             // Setpoint weight relaxation effect
    uint8_t dtermSetpointWeight;            // Setpoint weight for Dterm (0= measurement, 1= full error, 1 > aggressive derivative)
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t crash_dthreshold;              // dterm crash value
    uint16_t crash_gthreshold;              // gyro crash value
    uint16_t crash_setpoint_threshold;      // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    uint16_t crash_time;                    // ms
    uint16_t crash_delay;                   // ms
    uint8_t crash_recovery_angle;           // degrees
    uint8_t crash_recovery_rate;            // degree/second
//    pidCrashRecovery_e crash_recovery;      // off, on, on and beeps when it is in crash recovery mode
    uint16_t crash_limit_yaw;               // limits yaw errorRate, so crashes don't cause huge throttle increase
    uint16_t itermLimit;
} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

extern float axisPID_P[3], axisPID_I[3], axisPID_D[3];
extern struct pidProfile_s *currentPidProfile;
extern uint32_t targetPidLooptime;
/* PID END */

/* DEBUG START */
typedef enum {
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
} debugType_e;

extern uint8_t debugMode;
extern int16_t debug[DEBUG16_VALUE_COUNT];
/* DEBUG END */

/* SENSOR START */
bool sensors(uint32_t mask);

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6
} sensors_e;


typedef struct gyro_s {
    uint32_t targetLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
} gyro_t;

extern gyro_t gyro;

typedef struct accDev_s {
    //sensorAccInitFuncPtr initFn;                              // initialize function
    //sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    //busDevice_t bus;
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    char revisionCode;                                      // a revision code for the sensor, if known
    bool dataReady;
    //sensor_align_e accAlign;
    //mpuDetectionResult_t mpuDetectionResult;
    //mpuConfiguration_t mpuConfiguration;
} accDev_t;

typedef struct acc_s {
    accDev_t dev;
    uint32_t accSamplingInterval;
    int32_t accSmooth[XYZ_AXIS_COUNT];
    bool isAccelUpdatedAtLeastOnce;
} acc_t;

extern acc_t acc;

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
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    uint8_t vbatwarningcellvoltage;         // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    uint8_t vbatnotpresentcellvoltage;      // Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
    uint8_t lvcPercentage;                  // Percentage of throttle when lvc is triggered
    voltageMeterSource_e voltageMeterSource; // source of battery voltage meter used, either ADC or ESC

    // current
    currentMeterSource_e currentMeterSource; // source of battery current meter used, either ADC, Virtual or ESC
    uint16_t batteryCapacity;               // mAh

    // warnings / alerts
    bool useVBatAlerts;                     // Issue alerts based on VBat readings
    bool useConsumptionAlerts;              // Issue alerts based on total power consumption
    uint8_t consumptionWarningPercentage;   // Percentage of remaining capacity that should trigger a battery warning
    uint8_t vbathysteresis;                 // hysteresis for alarm, default 1 = 0.1V

    uint8_t vbatfullcellvoltage;            // Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)

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
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    //sensor_align_e mag_align;               // mag alignment
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    //ioTag_t interruptTag;
    //flightDynamicsTrims_t magZero;
} compassConfig_t;

PG_DECLARE(compassConfig_t, compassConfig);

typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    //sensor_align_e acc_align;               // acc alignment
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    //flightDynamicsTrims_t accZero;
    //rollAndPitchTrims_t accelerometerTrims;
} accelerometerConfig_t;

PG_DECLARE(accelerometerConfig_t, accelerometerConfig);

typedef struct barometerConfig_s {
    uint8_t baro_bustype;
    uint8_t baro_spi_device;
    //ioTag_t baro_spi_csn;                   // Also used as XCLR (positive logic) for BMP085
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // Barometer hardware to use
    uint8_t baro_sample_count;              // size of baro filter array
    uint16_t baro_noise_lpf;                // additional LPF to reduce baro noise
    uint16_t baro_cf_vel;                   // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    uint16_t baro_cf_alt;                   // apply CF to use ACC for height estimation
} barometerConfig_t;

PG_DECLARE(barometerConfig_t, barometerConfig);

typedef struct gyroConfig_s {
    //sensor_align_e gyro_align;              // gyro alignment
    uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_sync_denom;                  // Gyro sample divider
    uint8_t  gyro_lpf;                         // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t  gyro_soft_lpf_type;
    uint8_t  gyro_soft_lpf_hz;
    bool     gyro_use_32khz;
    uint8_t  gyro_to_use;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

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
    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t sbus_inversion;                 // default sbus (Futaba, FrSKY) is inverted. Support for uninverted OpenLRS (and modified FrSKY) receivers.
    uint8_t halfDuplex;                     // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
    uint8_t rx_spi_protocol;                // type of SPI RX protocol
                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
    uint32_t rx_spi_id;
    uint8_t rx_spi_rf_channel_count;
//    ioTag_t spektrum_bind_pin_override_ioTag;
//    ioTag_t spektrum_bind_plug_ioTag;
    uint8_t spektrum_sat_bind;              // number of bind pulses for Spektrum satellite receivers
    uint8_t spektrum_sat_bind_autoreset;    // whenever we will reset (exit) binding mode after hard reboot
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_invert;
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t rcInterpolation;
    uint8_t rcInterpolationChannels;
    uint8_t rcInterpolationInterval;
    uint8_t fpvCamAngleDegrees;             // Camera angle to be scaled into rc commands
    uint16_t airModeActivateThreshold;      // Throttle setpoint where airmode gets activated

    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    uint8_t max_aux_channel;
} rxConfig_t;

PG_DECLARE(rxConfig_t, rxConfig);

extern float rcCommand[4];
/* RX START */

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
/* FAILSAFE END */
