#include "platform.h"

//const char * const targetName = "ESP8266"; //__TARGET__;
//const char * const shortGitRevision = "0000000"; //__REVISION__;
const char * const targetName = "OMNIBUSF4SD"; //__TARGET__;
const char * const shortGitRevision = "e1c4b5ce1c4b5c"; //__REVISION__;
const char * const buildTime = __TIME__;
const char * const buildDate = __DATE__;

PG_RESET_TEMPLATE_DEF(serialConfig_t, serialConfig);
PG_RESET_TEMPLATE_DEF(mixerConfig_t, mixerConfig);
PG_RESET_TEMPLATE_DEF(motorConfig_t, motorConfig);
PG_RESET_TEMPLATE_DEF(featureConfig_t, featureConfig);
PG_RESET_TEMPLATE_DEF(flight3DConfig_t, flight3DConfig);
PG_RESET_TEMPLATE_DEF(armingConfig_t, armingConfig);
PG_RESET_TEMPLATE_DEF(rcControlsConfig_t, rcControlsConfig);
PG_RESET_TEMPLATE_DEF(pilotConfig_t, pilotConfig);
PG_RESET_TEMPLATE_DEF(systemConfig_t, systemConfig);
PG_RESET_TEMPLATE_DEF(pidConfig_t, pidConfig);
PG_RESET_TEMPLATE_DEF(batteryConfig_t, batteryConfig);
PG_RESET_TEMPLATE_DEF(compassConfig_t, compassConfig);
PG_RESET_TEMPLATE_DEF(accelerometerConfig_t, accelerometerConfig);
PG_RESET_TEMPLATE_DEF(barometerConfig_t, barometerConfig);
PG_RESET_TEMPLATE_DEF(gyroConfig_t, gyroConfig);
PG_RESET_TEMPLATE_DEF(currentSensorADCConfig_t, currentSensorADCConfig);
PG_RESET_TEMPLATE_DEF(rxConfig_t, rxConfig);

PG_RESET_TEMPLATE_ARRAY_DEF(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);
PG_RESET_TEMPLATE_ARRAY_DEF(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles);
PG_RESET_TEMPLATE_ARRAY_DEF(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

pidProfile_t *currentPidProfile;
boxBitmask_t rcModeActivationMask;
uint16_t flightModeFlags;
uint8_t stateFlags;
uint8_t armingFlags;
uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
gyro_t gyro;
acc_t acc = {
  .dev = {
    .acc_1G = 2048
  }
};
uint16_t rssi;
float axisPID_P[3], axisPID_I[3], axisPID_D[3];
float motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];
uint32_t targetPidLooptime;
float rcCommand[4];
float motorOutputHigh, motorOutputLow;

static serialPort_t _sp = {
    .txBufferSize = 128
};
static serialPortConfig_t _spc = {
    .blackbox_baudrateIndex = 5,
    .identifier = SERIAL_PORT_USART1
};
static uint32_t activeFeaturesLatch = 0;

int gcd(int num, int denom)
{
    if (denom == 0) return num;
    return gcd(denom, num % denom);
}

bool bitArrayGet(const void *array, unsigned bit)
{
    return BITARRAY_BIT_OP((uint32_t*)array, bit, &);
}

bool IS_RC_MODE_ACTIVE(boxId_e boxId)
{
    return bitArrayGet(&rcModeActivationMask, boxId);
}

bool feature(uint32_t mask)
{
    false;
    return activeFeaturesLatch & mask;
}

bool sensors(uint32_t mask)
{
    return true;
    //return enabledSensors & mask;
}

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
  return &_sp;
}

void mspSerialReleasePortIfAllocated(struct serialPort_s *serialPort)
{
  UNUSED(serialPort);
}

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
  return &_spc;
}

serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    return &_sp;
}

void closeSerialPort(serialPort_t *serialPort)
{
  UNUSED(serialPort);
}

void mspSerialAllocatePorts(void)
{
}

portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function)
{
    return PORTSHARING_UNUSED;
}

failsafePhase_e failsafePhase()
{
  return FAILSAFE_IDLE;
}

bool rxIsReceivingSignal(void)
{
  return true;
}

bool rxAreFlightChannelsValid(void)
{
  return true;
}

uint8_t getMotorCount()
{
  return MAX_SUPPORTED_MOTORS;
}

uint16_t getBatteryVoltageLatest(void)
{
    return 0;
}

int32_t getAmperageLatest(void)
{
    return 0;
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    return false;
}

static uint32_t armingBeepTimeUs = 0;

void setArmingBeepTimeMicros(uint32_t ts)
{
  armingBeepTimeUs = ts;
}

uint32_t getArmingBeepTimeMicros(void)
{
    return armingBeepTimeUs;
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

uint32_t castFloatBytesToInt(float f)
{
    union floatConvert_t {
        float f;
        uint32_t u;
    } floatConvert;
    floatConvert.f = f;
    return floatConvert.u;
}

uint32_t zigzagEncode(int32_t value)
{
    return (uint32_t)((value << 1) ^ (value >> 31));
}
