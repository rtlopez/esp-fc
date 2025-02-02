#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include <cstddef>
#include <cstdint>
#include <EscDriver.h>
#include "Debug_Espfc.h"
#include "ModelConfig.h"
#include "ModelState.h"
#include "Storage.h"
#include "Logger.h"
#include "Math/Utils.h"

namespace Espfc {

class Model
{
  public:
    Model()
    {
      initialize();
    }

    void initialize()
    {
      config = ModelConfig();
      #ifdef UNIT_TEST
      state = ModelState(); // FIXME: causes board wdt reset
      #endif
      //config.brobot();
    }

    /**
     * @deprecated use isModeActive
     */
    bool isActive(FlightMode mode) const
    {
      return isModeActive(mode);
    }

    bool isModeActive(FlightMode mode) const
    {
      return state.modeMask & (1 << mode);
    }

    bool hasChanged(FlightMode mode) const
    {
      return (state.modeMask & (1 << mode)) != (state.modeMaskPrev & (1 << mode));
    }

    void clearMode(FlightMode mode)
    {
      state.modeMaskPrev |= state.modeMask & (1 << mode);
      state.modeMask &= ~(1 << mode);
    }

    void updateModes(uint32_t mask)
    {
      state.modeMaskPrev = state.modeMask;
      state.modeMask = mask;
    }

    bool isSwitchActive(FlightMode mode) const
    {
      return state.modeMaskSwitch & (1 << mode);
    }

    void updateSwitchActive(uint32_t mask)
    {
      state.modeMaskSwitch = mask;
    }

    void disarm(DisarmReason r)
    {
      state.disarmReason = r;
      clearMode(MODE_ARMED);
      clearMode(MODE_AIRMODE);
      state.appQueue.send(Event(EVENT_DISARM));
    }

    /**
     * @deprecated use isFeatureActive
     */
    bool isActive(Feature feature) const
    {
      return isFeatureActive(feature);
    }

    bool isFeatureActive(Feature feature) const
    {
      return config.featureMask & feature;
    }

    bool isAirModeActive() const
    {
      return isModeActive(MODE_AIRMODE);// || isActive(FEATURE_AIRMODE);
    }

    bool isThrottleLow() const
    {
      return state.inputUs[AXIS_THRUST] < config.input.minCheck;
    }

    bool blackboxEnabled() const
    {
      // serial or flash
      return (config.blackboxDev == 3 || config.blackboxDev == 1) && config.blackboxPdenom > 0;
    }

    bool gyroActive() const /* IRAM_ATTR */
    {
      return state.gyroPresent && config.gyroDev != GYRO_NONE;
    }

    bool accelActive() const
    {
      return state.accelPresent && config.accelDev != GYRO_NONE;
    }

    bool magActive() const
    {
      return state.magPresent && config.magDev != MAG_NONE;
    }

    bool baroActive() const
    {
      return state.baroPresent && config.baroDev != BARO_NONE;
    }

    bool calibrationActive() const
    {
      return state.accelCalibrationState != CALIBRATION_IDLE || state.gyroCalibrationState != CALIBRATION_IDLE || state.magCalibrationState != CALIBRATION_IDLE;
    }

    void calibrateGyro()
    {
      state.gyroCalibrationState = CALIBRATION_START;
      if(accelActive())
      {
        state.accelCalibrationState = CALIBRATION_START;
      }
    }

    void calibrateMag()
    {
      state.magCalibrationState = CALIBRATION_START;
    }

    void finishCalibration()
    {
      if(state.gyroCalibrationState == CALIBRATION_SAVE)
      {
        //save();
        state.buzzer.push(BUZZER_GYRO_CALIBRATED);
        logger.info().log(F("GYRO BIAS")).log(degrees(state.gyroBias.x)).log(degrees(state.gyroBias.y)).logln(degrees(state.gyroBias.z));
      }
      if(state.accelCalibrationState == CALIBRATION_SAVE)
      {
        save();
        logger.info().log(F("ACCEL BIAS")).log(state.accelBias.x).log(state.accelBias.y).logln(state.accelBias.z);
      }
      if(state.magCalibrationState == CALIBRATION_SAVE)
      {
        save();
        logger.info().log(F("MAG BIAS")).log(state.magCalibrationOffset.x).log(state.magCalibrationOffset.y).logln(state.magCalibrationOffset.z);
        logger.info().log(F("MAG SCALE")).log(state.magCalibrationScale.x).log(state.magCalibrationScale.y).logln(state.magCalibrationScale.z);
      }
    }

    bool armingDisabled() const /* IRAM_ATTR */
    {
#if defined(ESPFC_DEV_PRESET_UNSAFE_ARMING)
      return false;
#else
      return state.armingDisabledFlags != 0;
#endif
    }

    void setArmingDisabled(ArmingDisabledFlags flag, bool value)
    {
      if(value) state.armingDisabledFlags |= flag;
      else state.armingDisabledFlags &= ~flag;
    }

    bool getArmingDisabled(ArmingDisabledFlags flag)
    {
      return state.armingDisabledFlags & flag;
    }

    void setOutputSaturated(bool val)
    {
      state.outputSaturated = val;
      for(size_t i = 0; i < 3; i++)
      {
        state.innerPid[i].outputSaturated = val;
        state.outerPid[i].outputSaturated = val;
      }
    }

    bool areMotorsRunning() const
    {
      size_t count = state.currentMixer.count;
      for(size_t i = 0; i < count; i++)
      {
        if(config.output.channel[i].servo) continue;
        if(state.outputDisarmed[i] != config.output.minCommand) return true;
        //if(state.outputUs[i] != config.output.minCommand) return true;
      }
      return false;
    }

    void inline setDebug(DebugMode mode, size_t index, int16_t value)
    {
      if(index >= 8) return;
      if(config.debugMode != mode) return;
      state.debug[index] = value;
    }

    Device::SerialDevice * getSerialStream(SerialPort i)
    {
      return state.serial[i].stream;
    }

    Device::SerialDevice * getSerialStream(SerialFunction sf)
    {
      for(size_t i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if(config.serial[i].functionMask & sf) return state.serial[i].stream;
      }
      return nullptr;
    }

    int getSerialIndex(SerialPortId id)
    {
      switch(id)
      {
#ifdef ESPFC_SERIAL_0
        case SERIAL_ID_UART_1: return SERIAL_UART_0;
#endif
#ifdef ESPFC_SERIAL_1
        case SERIAL_ID_UART_2: return SERIAL_UART_1;
#endif
#ifdef ESPFC_SERIAL_2
        case SERIAL_ID_UART_3: return SERIAL_UART_2;
#endif
#ifdef ESPFC_SERIAL_USB
        case SERIAL_ID_USB_VCP: return SERIAL_USB;
#endif
#ifdef ESPFC_SERIAL_SOFT_0
        case SERIAL_ID_SOFTSERIAL_1: return SERIAL_SOFT_0;
#endif
        default: break;
      }
      return -1;
    }

    uint16_t getRssi() const
    {
      size_t channel = config.input.rssiChannel;
      if(channel < 4 || channel > state.inputChannelCount) return 0;
      float value = state.input[channel - 1];
      return Math::clamp(lrintf(Math::map(value, -1.0f, 1.0f, 0.0f, 1023.0f)), 0l, 1023l);
    }

    int load()
    {
      logger.begin();
      #ifndef UNIT_TEST
      _storage.begin();
      _storageResult = _storage.load(config);
      #endif
      postLoad();
      return 1;
    }

    void save()
    {
      preSave();
      #ifndef UNIT_TEST
      _storageResult = _storage.write(config);
      #endif
    }

    void reload()
    {
      begin();
    }

    void reset()
    {
      initialize();
      save();
      reload();
    }

    void sanitize()
    {
      // for spi gyro allow full speed mode
      if (state.gyroDev && state.gyroDev->getBus()->isSPI())
      {
        state.gyroRate = Math::alignToClock(state.gyroClock, ESPFC_GYRO_SPI_RATE_MAX);
      }
      else
      {
        state.gyroRate = Math::alignToClock(state.gyroClock, ESPFC_GYRO_I2C_RATE_MAX);
        // first usage
        if(_storageResult == STORAGE_ERR_BAD_MAGIC || _storageResult == STORAGE_ERR_BAD_SIZE || _storageResult == STORAGE_ERR_BAD_VERSION)
        {
          config.loopSync = 1;
        }
      }

      int loopSyncMax = 1;
      //if(config.magDev != MAG_NONE || config.baroDev != BARO_NONE) loopSyncMax /= 2;

      config.loopSync = std::max((int)config.loopSync, loopSyncMax);
      state.loopRate = state.gyroRate / config.loopSync;

      config.output.protocol = ESC_PROTOCOL_SANITIZE(config.output.protocol);

      switch(config.output.protocol)
      {
        case ESC_PROTOCOL_BRUSHED:
          config.output.async = true;
          break;
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_PROSHOT:
          config.output.async = false;
          break;
      }

      if(config.output.async)
      {
        // for async limit pwm rate
        switch(config.output.protocol)
        {
          case ESC_PROTOCOL_PWM:
            config.output.rate = constrain(config.output.rate, 50, 480);
            break;
          case ESC_PROTOCOL_ONESHOT125:
            config.output.rate = constrain(config.output.rate, 50, 2000);
            break;
          case ESC_PROTOCOL_ONESHOT42:
            config.output.rate = constrain(config.output.rate, 50, 4000);
            break;
          case ESC_PROTOCOL_BRUSHED:
          case ESC_PROTOCOL_MULTISHOT:
            config.output.rate = constrain(config.output.rate, 50, 8000);
            break;
          default:
            config.output.rate = constrain(config.output.rate, 50, 2000);
            break;
        }
      }
      else
      {
        // for synced and standard PWM limit loop rate and pwm pulse width
        if(config.output.protocol == ESC_PROTOCOL_PWM && state.loopRate > 500)
        {
          config.loopSync = std::max(config.loopSync, (int8_t)((state.loopRate + 499) / 500)); // align loop rate to lower than 500Hz
          state.loopRate = state.gyroRate / config.loopSync;
          if(state.loopRate > 480 && config.output.maxThrottle > 1940)
          {
            config.output.maxThrottle = 1940;
          }
        }
        // for onshot125 limit loop rate to 2kHz
        if(config.output.protocol == ESC_PROTOCOL_ONESHOT125 && state.loopRate > 2000)
        {
          config.loopSync = std::max(config.loopSync, (int8_t)((state.loopRate + 1999) / 2000)); // align loop rate to lower than 2000Hz
          state.loopRate = state.gyroRate / config.loopSync;
        }
      }

      // sanitize throttle and motor limits
      if(config.output.throttleLimitType < 0 || config.output.throttleLimitType >= THROTTLE_LIMIT_TYPE_MAX) {
        config.output.throttleLimitType = THROTTLE_LIMIT_TYPE_NONE;
      }

      if(config.output.throttleLimitPercent < 1 || config.output.throttleLimitPercent > 100) {
        config.output.throttleLimitPercent = 100;
      }

      if(config.output.motorLimit < 1 || config.output.motorLimit > 100) {
        config.output.motorLimit = 100;
      }

      // configure serial ports
      constexpr uint32_t serialFunctionAllowedMask = SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_RX_SERIAL | SERIAL_FUNCTION_BLACKBOX | SERIAL_FUNCTION_TELEMETRY_FRSKY | SERIAL_FUNCTION_TELEMETRY_HOTT | SERIAL_FUNCTION_TELEMETRY_IBUS;
      uint32_t featureAllowMask = FEATURE_RX_SERIAL | FEATURE_RX_PPM | FEATURE_RX_SPI | FEATURE_SOFTSERIAL | FEATURE_MOTOR_STOP | FEATURE_TELEMETRY;// | FEATURE_AIRMODE;

      // allow dynamic filter only above 1k sampling rate
      if(state.loopRate >= DynamicFilterConfig::MIN_FREQ)
      {
        featureAllowMask |= FEATURE_DYNAMIC_FILTER;
      }

      config.featureMask &= featureAllowMask;

      for(int i = 0; i < SERIAL_UART_COUNT; i++) {
        config.serial[i].functionMask &= serialFunctionAllowedMask;
      }

      // only few beeper modes allowed
      config.buzzer.beeperMask &=
        1 << (BUZZER_GYRO_CALIBRATED - 1) |
        1 << (BUZZER_SYSTEM_INIT - 1) |
        1 << (BUZZER_RX_LOST - 1) |
        1 << (BUZZER_RX_SET - 1) |
        1 << (BUZZER_DISARMING - 1) |
        1 << (BUZZER_ARMING - 1) |
        1 << (BUZZER_BAT_LOW - 1);

        if(config.dynamicFilter.width > 6)
        {
          config.dynamicFilter.width = 6;
        }
    }

    void begin()
    {
      sanitize();

      // init timers
      // sample rate = clock / ( divider + 1)
      state.gyroTimer.setRate(state.gyroRate);
      int accelRate = Math::alignToClock(state.gyroTimer.rate, 500);
      state.accelTimer.setRate(state.gyroTimer.rate, state.gyroTimer.rate / accelRate);
      state.loopTimer.setRate(state.gyroTimer.rate, config.loopSync);
      state.mixerTimer.setRate(state.loopTimer.rate, config.mixerSync);
      int inputRate = Math::alignToClock(state.gyroTimer.rate, 1000);
      state.inputTimer.setRate(state.gyroTimer.rate, state.gyroTimer.rate / inputRate);
      state.actuatorTimer.setRate(50);
      state.dynamicFilterTimer.setRate(50);
      state.telemetryTimer.setInterval(config.telemetryInterval * 1000);
      state.stats.timer.setRate(3);
      if(magActive())
      {
        state.magTimer.setRate(state.magRate);
      }

      state.boardAlignment.init(VectorFloat(Math::toRad(config.boardAlignment[0]), Math::toRad(config.boardAlignment[1]), Math::toRad(config.boardAlignment[2])));

      const uint32_t gyroPreFilterRate = state.gyroTimer.rate;
      const uint32_t gyroFilterRate = state.loopTimer.rate;
      const uint32_t inputFilterRate = state.inputTimer.rate;
      const uint32_t pidFilterRate = state.loopTimer.rate;

      // configure filters
      for(size_t i = 0; i <= AXIS_YAW; i++)
      {
        if(isActive(FEATURE_DYNAMIC_FILTER))
        {
          for(size_t p = 0; p < (size_t)config.dynamicFilter.width; p++)
          {
            state.gyroDynNotchFilter[p][i].begin(FilterConfig(FILTER_NOTCH_DF1, 400, 380), gyroFilterRate);
          }
        }
        state.gyroNotch1Filter[i].begin(config.gyroNotch1Filter, gyroFilterRate);
        state.gyroNotch2Filter[i].begin(config.gyroNotch2Filter, gyroFilterRate);
        if(config.gyroDynLpfFilter.cutoff > 0)
        {
          state.gyroFilter[i].begin(FilterConfig((FilterType)config.gyroFilter.type, config.gyroDynLpfFilter.cutoff), gyroFilterRate);
        }
        else
        {
          state.gyroFilter[i].begin(config.gyroFilter, gyroFilterRate);
        }
        state.gyroFilter2[i].begin(config.gyroFilter2, gyroFilterRate);
        state.gyroFilter3[i].begin(config.gyroFilter3, gyroPreFilterRate);
        state.accelFilter[i].begin(config.accelFilter, gyroFilterRate);
        state.gyroImuFilter[i].begin(FilterConfig(FILTER_PT1, state.accelTimer.rate / 3), gyroFilterRate);
        for(size_t m = 0; m < RPM_FILTER_MOTOR_MAX; m++)
        {
          state.rpmFreqFilter[m].begin(FilterConfig(FILTER_PT1, config.rpmFilterFreqLpf), gyroFilterRate);
          for(size_t n = 0; n < config.rpmFilterHarmonics; n++)
          {
            int center = Math::mapi(m * RPM_FILTER_HARMONICS_MAX + n, 0, RPM_FILTER_MOTOR_MAX * config.rpmFilterHarmonics, config.rpmFilterMinFreq, gyroFilterRate / 2);
            state.rpmFilter[m][n][i].begin(FilterConfig(FILTER_NOTCH_DF1, center, center * 0.98f), gyroFilterRate);
          }
        }
        if(magActive())
        {
          state.magFilter[i].begin(config.magFilter, state.magTimer.rate);
        }
      }

      for(size_t i = 0; i < 4; i++)
      {
        if (config.input.filterType == INPUT_FILTER)
        {
          state.inputFilter[i].begin(config.input.filter, inputFilterRate);
        }
        else
        {
          state.inputFilter[i].begin(FilterConfig(FILTER_PT3, 25), inputFilterRate);
        }
      }

      // ensure disarmed pulses
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        state.outputDisarmed[i] = config.output.channel[i].servo ? config.output.channel[i].neutral : config.output.minCommand; // ROBOT
      }

      state.buzzer.beeperMask = config.buzzer.beeperMask;

      // configure PIDs
      float pidScale[] = { 1.f, 1.f, 1.f };
      if(config.mixerType == FC_MIXER_GIMBAL)
      {
        pidScale[AXIS_YAW] = 0.2f; // ROBOT
        pidScale[AXIS_PITCH] = 20.f; // ROBOT
      }

      for(size_t i = 0; i <= AXIS_YAW; i++) // rpy
      {
        const PidConfig& pc = config.pid[i];
        Control::Pid& pid = state.innerPid[i];
        pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[i];
        pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[i];
        pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[i];
        pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[i];
        pid.iLimit = config.itermLimit * 0.01f;
        pid.oLimit = 0.66f;
        pid.rate = state.loopTimer.rate;
        pid.dtermNotchFilter.begin(config.dtermNotchFilter, pidFilterRate);
        if(config.dtermDynLpfFilter.cutoff > 0) {
          pid.dtermFilter.begin(FilterConfig((FilterType)config.dtermFilter.type, config.dtermDynLpfFilter.cutoff), pidFilterRate);
        } else {
          pid.dtermFilter.begin(config.dtermFilter, pidFilterRate);
        }
        pid.dtermFilter2.begin(config.dtermFilter2, pidFilterRate);
        pid.ftermFilter.begin(config.input.filterDerivative, pidFilterRate);
        pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, config.itermRelaxCutoff), pidFilterRate);
        if(i == AXIS_YAW) {
          pid.itermRelax = config.itermRelax == ITERM_RELAX_RPY || config.itermRelax == ITERM_RELAX_RPY_INC ? config.itermRelax : ITERM_RELAX_OFF;
          pid.ptermFilter.begin(config.yawFilter, pidFilterRate);
        } else {
          pid.itermRelax = config.itermRelax;
        }
        pid.begin();
      }

      for(size_t i = 0; i < AXIS_YAW; i++)
      {
        PidConfig& pc = config.pid[FC_PID_LEVEL];
        Control::Pid& pid = state.outerPid[i];
        pid.Kp = (float)pc.P * LEVEL_PTERM_SCALE;
        pid.Ki = (float)pc.I * LEVEL_ITERM_SCALE;
        pid.Kd = (float)pc.D * LEVEL_DTERM_SCALE;
        pid.Kf = (float)pc.F * LEVEL_FTERM_SCALE;
        pid.iLimit = Math::toRad(config.angleRateLimit) * 0.1f;
        pid.oLimit = Math::toRad(config.angleRateLimit);
        pid.rate = state.loopTimer.rate;
        pid.ptermFilter.begin(config.levelPtermFilter, pidFilterRate);
        //pid.iLimit = 0.3f; // ROBOT
        //pid.oLimit = 1.f;  // ROBOT
        pid.begin();
      }
      state.customMixer = MixerConfig(config.customMixerCount, config.customMixes);

      // override temporary
      //state.telemetryTimer.setRate(100);
    }

    void postLoad()
    {
      // load current sensor calibration
      for(size_t i = 0; i <= AXIS_YAW; i++)
      {
        state.gyroBias.set(i, config.gyroBias[i] / 1000.0f);
        state.accelBias.set(i, config.accelBias[i] / 1000.0f);
        state.magCalibrationOffset.set(i, config.magCalibrationOffset[i] / 10.0f);
        state.magCalibrationScale.set(i, config.magCalibrationScale[i] / 1000.0f);
      }
    }

    void preSave()
    {
      // store current sensor calibration
      for(size_t i = 0; i < 3; i++)
      {
        config.gyroBias[i] = lrintf(state.gyroBias[i] * 1000.0f);
        config.accelBias[i] = lrintf(state.accelBias[i] * 1000.0f);
        config.magCalibrationOffset[i] = lrintf(state.magCalibrationOffset[i] * 10.0f);
        config.magCalibrationScale[i] = lrintf(state.magCalibrationScale[i] * 1000.0f);
      }
    }

    ModelState state;
    ModelConfig config;
    Logger logger;

    void logStorageResult()
    {
#ifndef UNIT_TEST
      logger.info().log(F("F_CPU")).logln(F_CPU);
      switch(_storageResult)
      {
        case STORAGE_LOAD_SUCCESS:    logger.info().logln(F("EEPROM loaded")); break;
        case STORAGE_SAVE_SUCCESS:    logger.info().logln(F("EEPROM saved")); break;
        case STORAGE_ERR_BAD_MAGIC:   logger.err().logln(F("EEPROM wrong magic")); break;
        case STORAGE_ERR_BAD_VERSION: logger.err().logln(F("EEPROM wrong version")); break;
        case STORAGE_ERR_BAD_SIZE:    logger.err().logln(F("EEPROM wrong size")); break;
        case STORAGE_NONE:
        default:
          logger.err().logln(F("EEPROM unknown")); break;
      }
#endif
    }

  private:
    #ifndef UNIT_TEST
    Storage _storage;
    #endif
    StorageResult _storageResult;
};

}

#endif
