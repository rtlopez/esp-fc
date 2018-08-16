#ifndef _ESPFC_MODEL_H_
#define _ESPFC_MODEL_H_

#include <cstddef>
#include <cstdint>

#include "Debug.h"
#include "ModelConfig.h"
#include "ModelState.h"
#include "Storage.h"
#include "Logger.h"
#include "Math.h"
#include "EspGpio.h"
#include "EscDriver.h"

#if defined(ESP32)
#define ESPFC_GUARD 0
#elif defined(ESP8266)
#define ESPFC_GUARD 1
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
      //config.brobot();
    }

    bool isActive(FlightMode mode) const
    {
      return state.modeMask & (1 << mode);
    }

    bool isSwitchActive(FlightMode mode) const
    {
      return state.modeMaskNew & (1 << mode);
    }

    bool hasChanged(FlightMode mode) const
    {
      return (state.modeMask & (1 << mode)) != (state.modeMaskPrev & (1 << mode));
    }

    bool isActive(Feature feature) const
    {
      return config.featureMask & feature;
    }

    bool isThrottleLow() const
    {
      return state.inputUs[AXIS_THRUST] < config.input.minCheck;
    }

    bool blackboxEnabled() const
    {
      return config.blackboxDev == 3 && config.blackboxPdenom > 0;
    }

    bool gyroActive() const ICACHE_RAM_ATTR
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
      return state.sensorCalibration || state.accelBiasSamples > 0 || state.gyroBiasSamples > 0;
    }

    void calibrate()
    {
      state.sensorCalibration = true;
      state.gyroBiasSamples  = 2 * state.gyroTimer.rate;
      if(accelActive())
      {
        state.accelBiasSamples = 2 * state.gyroTimer.rate;
        state.accelBias.z += 1.f;
      }
    }

    void finishCalibration()
    {
      if(state.sensorCalibration && state.accelBiasSamples == 0 && state.gyroBiasSamples <= 0)
      {
        state.sensorCalibration = false;
        save();
      }
    }

    bool armingDisabled() const ICACHE_RAM_ATTR
    {
      return state.armingDisabledFlags != 0;
    }

    SerialDevice * getSerialStream(SerialPort i)
    {
      return state.serial[i].stream;
    }

    SerialDevice * getSerialStream(SerialFunction sf)
    {
      for(size_t i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if(config.serial[i].functionMask & sf) return state.serial[i].stream;
      }
      return nullptr;
    }

    void begin()
    {
      logger.begin();
      _storage.begin();
      load();
      sanitize();
    }

    void sanitize()
    {
      int gyroSyncMax = 4; // max 2khz
      if(state.gyroBus == BUS_SPI)
      {
        gyroSyncMax = 1; // max 8k
        //if(config.accelDev != GYRO_NONE) gyroSyncMax /= 2; // max 4khz
        //if(config.magDev != MAG_NONE || config.baroDev != BARO_NONE) gyroSyncMax /= 4; // max 2khz
      }
      else
      {
        //if(config.accelDev != GYRO_NONE) gyroSyncMax = 8; // max 1khz
        //if(config.magDev != MAG_NONE || config.baroDev != BARO_NONE) gyroSyncMax = 16; // max 500hz
      }

      config.gyroSync = std::max((int)config.gyroSync, gyroSyncMax); // max 1khz
      state.gyroClock = 8000;
      if(config.gyroDlpf != GYRO_DLPF_256)
      {
        state.gyroClock = 1000;
        config.gyroSync = ((config.gyroSync + 7) / 8) * 8;
      }
      int gyroSampleRate = 8000 / config.gyroSync;
      state.gyroDivider = (state.gyroClock / (gyroSampleRate + 1)) + 1;

      config.output.protocol = ESC_PROTOCOL_SANITIZE(config.output.protocol);

      switch(config.output.protocol)
      {
        case ESC_PROTOCOL_BRUSHED:
          config.output.async = true;
          break;
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_DSHOT1200:
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
        if(config.output.protocol == ESC_PROTOCOL_PWM && gyroSampleRate > 500)
        {
          config.loopSync = std::max(config.loopSync, (int8_t)((gyroSampleRate + 499) / 500)); // align loop rate to lower than 500Hz
          int loopRate = gyroSampleRate / config.loopSync;
          if(loopRate > 480 && config.output.maxThrottle > 1950)
          {
            config.output.maxThrottle = 1940;
          }
        }
      }

      // configure serial ports
      uint32_t serialFunctionAllowedMask = SERIAL_FUNCTION_MSP | SERIAL_FUNCTION_BLACKBOX | SERIAL_FUNCTION_TELEMETRY_FRSKY | SERIAL_FUNCTION_TELEMETRY_HOTT;
      uint32_t featureAllowMask = FEATURE_RX_PPM | FEATURE_MOTOR_STOP | FEATURE_TELEMETRY;

      // allow dynamic filter only above 1k sampling rate
      if(gyroSampleRate >= 1000)
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
      config.serial[SERIAL_WIFI_0].functionMask &= serialFunctionAllowedMask & ~FEATURE_RX_SERIAL;
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
      state.gyroTimer.setRate(state.gyroClock / state.gyroDivider);
      state.loopTimer.setRate(state.gyroTimer.rate, config.loopSync);
      state.mixerTimer.setRate(state.loopTimer.rate, config.mixerSync);
      state.actuatorTimer.setRate(25); // 25 hz
      state.telemetryTimer.setInterval(config.telemetryInterval * 1000);
      state.stats.timer.setRate(10);
      state.accelTimer.setRate(constrain(state.gyroTimer.rate, 100, 200));
      state.accelTimer.setInterval(state.accelTimer.interval - 20);
      state.serialTimer.setRate(1000);

      // configure calibration
      state.gyroBiasAlpha = 5.0f / state.gyroTimer.rate;
      state.accelBiasAlpha = 5.0f / state.accelTimer.rate;
      state.gyroBiasSamples = 2 * state.gyroTimer.rate; // start gyro calibration

      // load sensor calibration data
      for(size_t i = 0; i <= AXIS_YAW; i++)
      {
        state.gyroBias.set(i, config.gyroBias[i] / 1000.0f);
        state.accelBias.set(i, config.accelBias[i] / 1000.0f);
        state.magCalibrationOffset.set(i, config.magCalibrationOffset[i] / 1000.0f);
        state.magCalibrationScale.set(i, config.magCalibrationScale[i] / 1000.0f);
      }

      for(size_t i = 0; i <= AXIS_YAW; i++)
      {
        state.gyroAnalyzer[i].begin(state.gyroTimer.rate);
        state.gyroDynamicFilter[i].begin(FilterConfig(FILTER_NOTCH_DF1, 400, 300), state.gyroTimer.rate);
        state.gyroNotch1Filter[i].begin(config.gyroNotch1Filter, state.gyroTimer.rate);
        state.gyroNotch2Filter[i].begin(config.gyroNotch2Filter, state.gyroTimer.rate);
        state.gyroFilter[i].begin(config.gyroFilter, state.gyroTimer.rate);
        state.gyroFilter2[i].begin(config.gyroFilter2, state.gyroTimer.rate);
        state.gyroFilter3[i].begin(config.gyroFilter3, state.gyroTimer.rate);
        
        state.accelFilter[i].begin(config.accelFilter, state.accelTimer.rate);
        state.magFilter[i].begin(config.magFilter, state.gyroTimer.rate);
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
        PidConfig * pc = &config.pid[i];
        state.innerPid[i].configure(
          pc->P * PTERM_SCALE * pidScale[i],
          pc->I * ITERM_SCALE * pidScale[i],
          pc->D * DTERM_SCALE * pidScale[i],
          0.15f,
          config.dtermSetpointWeight * 0.01f,
          0.5f
        );
        state.innerPid[i].dtermFilter.begin(config.dtermFilter, state.loopTimer.rate);
        state.innerPid[i].dtermFilter2.begin(config.dtermFilter2, state.loopTimer.rate);
        state.innerPid[i].dtermNotchFilter.begin(config.dtermNotchFilter, state.loopTimer.rate);
        if(i == AXIS_YAW) state.innerPid[i].ptermFilter.begin(config.yawFilter, state.loopTimer.rate);
        else state.innerPid[i].ptermFilter.begin(); // no filter
      }

      for(size_t i = 0; i <= AXIS_YAW; i++)
      {
        PidConfig * pc = &config.pid[PID_LEVEL];
        state.outerPid[i].configure(
          pc->P * LEVEL_PTERM_SCALE,
          pc->I * LEVEL_ITERM_SCALE,
          pc->D * LEVEL_DTERM_SCALE,
          radians(config.angleRateLimit) * 0.1f,
          0,
          radians(config.angleRateLimit)
        );

        //state.outerPid[i].iLimit = 0.3f; // ROBOT
        //state.outerPid[i].dGamma = config.dtermSetpointWeight / 100.0f;  // ROBOT
        //state.outerPid[i].oLimit = 1.f;  // ROBOT

        state.outerPid[i].dtermFilter.begin(config.dtermFilter, state.loopTimer.rate);
        state.outerPid[i].dtermNotchFilter.begin(config.dtermNotchFilter, state.loopTimer.rate);
        state.outerPid[i].ptermFilter.begin(config.levelPtermFilter, state.loopTimer.rate);
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

    void preSave()
    {
      // store current sensor calibration
      for(size_t i = 0; i < 3; i++)
      {
        config.gyroBias[i] = lrintf(state.gyroBias[i] * 1000.0f);
        config.accelBias[i] = lrintf(state.accelBias[i] * 1000.0f);
        config.magCalibrationOffset[i] = lrintf(state.magCalibrationOffset[i] * 1000.0f);
        config.magCalibrationScale[i] = lrintf(state.magCalibrationScale[i] * 1000.0f);
      }
    }

    int load()
    {
      _storage.load(config, logger);
      return 1;
    }

    void save()
    {
      preSave();
      _storage.write(config, logger);
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

  private:
    Storage _storage;
};

}

#endif
