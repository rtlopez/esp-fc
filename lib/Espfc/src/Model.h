#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include <cstddef>
#include <cstdint>

#include <EspGpio.h>
#include <EscDriver.h>
#include <Hal.h>

#include "Debug_Espfc.h"
#include "ModelConfig.h"
#include "ModelState.h"
#include "Storage.h"
#include "Logger.h"
#include "Math/Utils.h"

#if defined(ESP8266)
#define ESPFC_GUARD 1
#else
#define ESPFC_GUARD 0
#endif

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

    void disarm()
    {
      clearMode(MODE_ARMED);
      clearMode(MODE_AIRMODE);
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
      return isActive(MODE_AIRMODE);// || isActive(FEATURE_AIRMODE);
    }

    bool isThrottleLow() const
    {
      return state.inputUs[AXIS_THRUST] < config.input.minCheck;
    }

    bool blackboxEnabled() const
    {
      return config.blackboxDev == 3 && config.blackboxPdenom > 0;
    }

    bool gyroActive() const IRAM_ATTR
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
        save();
        state.buzzer.push(BEEPER_GYRO_CALIBRATED);
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

    bool armingDisabled() const IRAM_ATTR
    {
      return state.armingDisabledFlags != 0;
    }

    void setArmingDisabled(ArmingDisabledFlags flag, bool value)
    {
      if(value) state.armingDisabledFlags |= flag;
      else state.armingDisabledFlags &= ~flag;
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
        case SERIAL_ID_UART_1: return SERIAL_UART_0;
        case SERIAL_ID_UART_2: return SERIAL_UART_1;
      #if defined(ESP32)
        case SERIAL_ID_UART_3: return SERIAL_UART_2;
        case SERIAL_ID_SOFTSERIAL_1: return SERIAL_SOFT_0;
      #elif defined(ESP8266)
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
      return Math::clamp(lrintf(Math::map(value, -1.0f, 1.0f, 0, 1023)), 0l, 1023l);
    }

    void begin()
    {
      logger.begin();
      #ifndef UNIT_TEST
      _storage.begin();
      #endif
      load();
      sanitize();
    }

    void sanitize()
    {
      int gyroSyncMax = 1; // max 8kHz
      #if defined(ESP8266)
        gyroSyncMax = 4; // max 2khz
      #endif
      //if(config.accelDev != GYRO_NONE) gyroSyncMax /= 2;
      //if(config.magDev != MAG_NONE || config.baroDev != BARO_NONE) gyroSyncMax /= 2;
      config.gyroSync = std::max((int)config.gyroSync, gyroSyncMax);

      switch(config.gyroDlpf)
      {
        case GYRO_DLPF_256:
        case GYRO_DLPF_EX:
          state.gyroClock = 8000;
          state.gyroRate = 8000 / config.gyroSync;
          state.gyroDivider = (state.gyroClock / (state.gyroRate + 1)) + 1;
          break;
        default:
          config.gyroSync = ((config.gyroSync + 7) / 8) * 8; // multiply 8x and round (BF GUI uses this convention)
          state.gyroClock = 1000;
          state.gyroRate = 8000 / config.gyroSync;
          state.gyroDivider = (state.gyroClock / (state.gyroRate + 1)) + 1;
          break;
      }

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
        //case ESC_PROTOCOL_DSHOT1200:
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
      uint32_t serialFunctionAllowedMask = SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_BLACKBOX | SERIAL_FUNCTION_TELEMETRY_FRSKY | SERIAL_FUNCTION_TELEMETRY_HOTT;
      uint32_t featureAllowMask = FEATURE_RX_PPM | FEATURE_MOTOR_STOP | FEATURE_TELEMETRY;// | FEATURE_AIRMODE;

      // allow dynamic filter only above 1k sampling rate
      if(state.gyroRate >= 1000)
      {
        featureAllowMask |= FEATURE_DYNAMIC_FILTER;
      }

      if(config.softSerialGuard || !ESPFC_GUARD)
      {
        featureAllowMask |= FEATURE_SOFTSERIAL;
      }
      if(config.serialRxGuard || !ESPFC_GUARD)
      {
        featureAllowMask |= FEATURE_RX_SERIAL;
        serialFunctionAllowedMask |= SERIAL_FUNCTION_RX_SERIAL;
      }
      config.featureMask &= featureAllowMask;

#if defined(ESP32)
      config.serial[SERIAL_UART_0].functionMask &= serialFunctionAllowedMask;
      config.serial[SERIAL_UART_1].functionMask &= serialFunctionAllowedMask;
      config.serial[SERIAL_UART_2].functionMask &= serialFunctionAllowedMask;
      config.serial[SERIAL_SOFT_0].functionMask &= serialFunctionAllowedMask & ~FEATURE_RX_SERIAL;
#elif defined(ESP8266)
      config.serial[SERIAL_UART_0].functionMask &= serialFunctionAllowedMask;
      config.serial[SERIAL_UART_1].functionMask &= serialFunctionAllowedMask;
      config.serial[SERIAL_SOFT_0].functionMask &= serialFunctionAllowedMask;
      //config.serial[SERIAL_SOFT_0].functionMask &= ~SERIAL_FUNCTION_RX_SERIAL;  // disallow
      //config.serial[SERIAL_SOFT_0].functionMask |= SERIAL_FUNCTION_RX_SERIAL; // force
#endif
      //config.featureMask |= FEATURE_RX_PPM; // force ppm
      //config.featureMask &= ~FEATURE_RX_PPM; // disallow ppm

      config.serial[SERIAL_UART_0].functionMask |= SERIAL_FUNCTION_MSP; // msp always enabled on uart0

      // only few beeper modes allowed
      config.buzzer.beeperMask &=
        1 << (BEEPER_GYRO_CALIBRATED - 1) |
        1 << (BEEPER_SYSTEM_INIT - 1) |
        1 << (BEEPER_RX_LOST - 1) |
        1 << (BEEPER_RX_SET - 1) |
        1 << (BEEPER_DISARMING - 1) |
        1 << (BEEPER_ARMING - 1) |
        1 << (BEEPER_BAT_LOW - 1);
    }

    void update()
    {
      // init timers
      // sample rate = clock / ( divider + 1)
      state.gyroTimer.setRate(state.gyroRate);
      state.accelTimer.setRate(constrain(state.gyroTimer.rate, 100, 500));
      state.accelTimer.setInterval(state.accelTimer.interval - 10);
      //state.accelTimer.setRate(state.gyroTimer.rate, 2);
      state.loopTimer.setRate(state.gyroTimer.rate, config.loopSync);
      state.mixerTimer.setRate(state.loopTimer.rate, config.mixerSync);
      state.actuatorTimer.setRate(25); // 25 hz
      state.dynamicFilterTimer.setRate(50);
      state.telemetryTimer.setInterval(config.telemetryInterval * 1000);
      state.stats.timer.setRate(10);
      state.serialTimer.setRate(1000);
      if(magActive())
      {
        state.magTimer.setRate(state.magRate);
      }

      // configure filters
      for(size_t i = 0; i <= AXIS_YAW; i++)
      {
        state.gyroAnalyzer[i].begin(state.gyroTimer.rate, config.dynamicFilter);
        if(isActive(FEATURE_DYNAMIC_FILTER)) {
          state.gyroDynamicFilter[i].begin(FilterConfig(FILTER_NOTCH_DF1, 400, 300), state.gyroTimer.rate);
          if(config.dynamicFilter.width > 0) {
            state.gyroDynamicFilter2[i].begin(FilterConfig(FILTER_NOTCH_DF1, 400, 300), state.gyroTimer.rate);
          }
        }
        state.gyroNotch1Filter[i].begin(config.gyroNotch1Filter, state.gyroTimer.rate);
        state.gyroNotch2Filter[i].begin(config.gyroNotch2Filter, state.gyroTimer.rate);
        if(config.gyroDynLpfFilter.cutoff > 0) {
          state.gyroFilter[i].begin(FilterConfig((FilterType)config.gyroFilter.type, config.gyroDynLpfFilter.cutoff), state.gyroTimer.rate);
        } else {
          state.gyroFilter[i].begin(config.gyroFilter, state.gyroTimer.rate);
        }
        state.gyroFilter2[i].begin(config.gyroFilter2, state.gyroTimer.rate);
        state.gyroFilter3[i].begin(config.gyroFilter3, state.gyroTimer.rate);
        state.accelFilter[i].begin(config.accelFilter, state.accelTimer.rate);
        state.gyroFilterImu[i].begin(FilterConfig(FILTER_PT1, state.accelTimer.rate / 2), state.gyroTimer.rate);
        if(magActive())
        {
          state.magFilter[i].begin(config.magFilter, state.magTimer.rate);
        }
      }

      for(size_t i = 0; i < 4; i++)
      {
        state.inputFilter[i].begin(FilterConfig(FILTER_PT1, 30), state.gyroTimer.rate);
      }

      // ensure disarmed pulses
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        state.outputDisarmed[i] = config.output.channel[i].servo ? config.output.channel[i].neutral : config.output.minCommand; // ROBOT
      }

      state.buzzer.beeperMask = config.buzzer.beeperMask;

      // configure PIDs
      float pidScale[] = { 1.f, 1.f, 1.f };
      if(config.mixerType == MIXER_GIMBAL)
      {
        pidScale[AXIS_YAW] = 0.2f; // ROBOT
        pidScale[AXIS_PITCH] = 20.f; // ROBOT
      }

      for(size_t i = 0; i <= AXIS_YAW; i++) // rpy
      {
        const PidConfig& pc = config.pid[i];
        Pid& pid = state.innerPid[i];
        pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[i];
        pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[i];
        pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[i];
        pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[i];
        pid.iLimit = 0.15f;
        pid.oLimit = 0.5f;
        pid.rate = state.loopTimer.rate;
        pid.dtermNotchFilter.begin(config.dtermNotchFilter, state.loopTimer.rate);
        if(config.dtermDynLpfFilter.cutoff > 0) {
          pid.dtermFilter.begin(FilterConfig((FilterType)config.dtermFilter.type, config.dtermDynLpfFilter.cutoff), state.loopTimer.rate);
        } else {
          pid.dtermFilter.begin(config.dtermFilter, state.loopTimer.rate);
        }
        pid.dtermFilter2.begin(config.dtermFilter2, state.loopTimer.rate);
        pid.ftermFilter.begin(FilterConfig(FILTER_PT1, 20), state.loopTimer.rate);
        if(i == AXIS_YAW) pid.ptermFilter.begin(config.yawFilter, state.loopTimer.rate);
        pid.begin();
      }

      for(size_t i = 0; i < AXIS_YAW; i++)
      {
        PidConfig& pc = config.pid[PID_LEVEL];
        Pid& pid = state.outerPid[i];
        pid.Kp = (float)pc.P * LEVEL_PTERM_SCALE;
        pid.Ki = (float)pc.I * LEVEL_ITERM_SCALE;
        pid.Kd = (float)pc.D * LEVEL_DTERM_SCALE;
        pid.Kf = (float)pc.F * LEVEL_FTERM_SCALE;
        pid.iLimit = Math::toRad(config.angleRateLimit) * 0.1f;
        pid.oLimit = Math::toRad(config.angleRateLimit);
        pid.rate = state.loopTimer.rate;
        pid.ptermFilter.begin(config.levelPtermFilter, state.loopTimer.rate);
        //pid.iLimit = 0.3f; // ROBOT
        //pid.oLimit = 1.f;  // ROBOT
        pid.begin();
      }
      state.customMixer = MixerConfig(config.customMixerCount, config.customMixes);

      //config.scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_PITCH); // ROBOT
      //config.scaler[1].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_YAW);   // ROBOT
      //config.scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_PITCH); // ROBOT
      //config.scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_PITCH); // ROBOT
      //config.scaler[0].dimension = (ScalerDimension)(ACT_OUTER_P | ACT_AXIS_PITCH); // ROBOT
      //config.scaler[1].dimension = (ScalerDimension)(ACT_OUTER_I | ACT_AXIS_PITCH); // ROBOT
      //config.scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_YAW | ACT_AXIS_ROLL | ACT_AXIS_PITCH);

      state.telemetry = config.telemetry;
      state.baroAlititudeBiasSamples = 200;

      // override temporary
      //state.telemetry = true;
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

    int load()
    {
      #ifndef UNIT_TEST
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
      sanitize();
      update();
    }

    void reset()
    {
      initialize();
      save();
      sanitize();
      update();
    }

    ModelState state;
    ModelConfig config;
    Logger logger;

    void logStorageResult()
    {
#ifndef UNIT_TEST
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
    StorageResult _storageResult;
    #endif
};

}

#endif
