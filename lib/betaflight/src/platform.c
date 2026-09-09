#include "platform.h"

const char * const targetName = ESPFC_TARGET;
const char * const targetVersion = STR(ESPFC_VERSION);
const char * const shortGitRevision = STR(ESPFC_REVISION);
const char * const buildTime = __TIME__;
const char * const buildDate = __DATE__;
const char * flightControllerIdentifier = "BTFL";
const char * boardIdentifier = "ESPF";
uint32_t systemUniqueId[3] = { 0 };

PG_RESET_TEMPLATE_DEF(serialConfig_t, serialConfig);
PG_RESET_TEMPLATE_DEF(mixerConfig_t, mixerConfig);
PG_RESET_TEMPLATE_DEF(motorConfig_t, motorConfig);
PG_RESET_TEMPLATE_DEF(rpmFilterConfig_t, rpmFilterConfig);
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
PG_RESET_TEMPLATE_DEF(positionConfig_t, positionConfig);
PG_RESET_TEMPLATE_DEF(dynNotchConfig_t, dynNotchConfig);
PG_RESET_TEMPLATE_DEF(gpsConfig_t, gpsConfig);
PG_RESET_TEMPLATE_DEF(autopilotConfig_t, autopilotConfig);

PG_RESET_TEMPLATE_ARRAY_DEF(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);
PG_RESET_TEMPLATE_ARRAY_DEF(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles);
PG_RESET_TEMPLATE_ARRAY_DEF(batteryProfile_t, BATTERY_PROFILE_COUNT, batteryProfiles);
PG_RESET_TEMPLATE_ARRAY_DEF(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

pidProfile_t *currentPidProfile;
batteryProfile_t *currentBatteryProfile;
boxBitmask_t rcModeActivationMask;
boxBitmask_t rcModeActivationPresent;
uint8_t stateFlags;
uint8_t armingFlags;
uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t activePidLoopDenom = 1;
gyro_t gyro;
acc_t acc = {
  .dev = {
    .acc_1G = 2048
  }
};
mag_t mag;
baro_t baro;
uint16_t rssi;
pidAxisData_t pidData[3];
float motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];
uint32_t targetPidLooptime;
float rcCommand[4];
bool useDshotTelemetry;
gpsLocation_t GPS_home_llh;
gpsSolutionData_t gpsSol;
quaternion_t imuAttitudeQuaternion;

// Days from start of year for each month (non-leap year first row, leap year second row)
static const uint16_t gpsMonthDays[2][12] = {
    { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 }, // Non-leap year
    { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 }  // Leap year
};

// Convert date/time to Unix seconds (UTC)
static int64_t dateTimeToUnixSeconds(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    int32_t days = 0;
    for (int y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }
    int isLeap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 1 : 0;
    days += gpsMonthDays[isLeap][month - 1] + (day - 1);
    return (int64_t)days * 86400 + hour * 3600 + min * 60 + sec;
}

// Convert gpsDateTime_t to Unix epoch seconds; returns 0 if not valid
uint32_t gpsDateTimeToEpoch(const gpsDateTime_t *dt)
{
    if (!dt || !dt->valid) {
        return 0;
    }

    if (dt->year < 1980 || dt->month < 1 || dt->month > 12 || dt->day < 1 ||
        dt->day > 31 || dt->hour > 23 || dt->min > 59 || dt->sec > 60) {
        return 0;
    }

    return (uint32_t)dateTimeToUnixSeconds(dt->year, dt->month, dt->day, dt->hour, dt->min, dt->sec);
}

const char* const lookupTableMixerType[] = {
    "LEGACY", "LINEAR", "DYNAMIC", "EZLANDING",
};

bool bitArrayGet(const void *array, unsigned bit)
{
    return BITARRAY_BIT_OP((uint32_t*)array, bit, &);
}

void bitArraySet(void *array, unsigned bit)
{
    BITARRAY_BIT_OP((uint32_t*)array, bit, |=);
}

void bitArrayClr(void *array, unsigned bit)
{
    BITARRAY_BIT_OP((uint32_t*)array, bit, &=~);
}

bool IS_RC_MODE_ACTIVE(boxId_e boxId)
{
    return bitArrayGet(&rcModeActivationMask, boxId);
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    return bitArrayGet(&rcModeActivationPresent, modeId);
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
