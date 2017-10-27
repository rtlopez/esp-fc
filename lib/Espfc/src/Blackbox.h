#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_

#include "Model.h"
#include "Hardware.h"

extern "C" {
#include "blackbox/blackbox.h"

static HardwareSerial * blackboxSerial = NULL;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
  UNUSED(instance);
  if(blackboxSerial) blackboxSerial->write(ch);
}

void serialWriteInit(HardwareSerial * serial)
{
  blackboxSerial = serial;
}

uint32_t serialTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    if(!blackboxSerial) return 0;
    return blackboxSerial->availableForWrite();
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    if(!blackboxSerial) return false;
    return blackboxSerial->availableForWrite() > 127;
}

}

namespace Espfc {

class Blackbox
{
  public:
    Blackbox(Model& model): _model(model) {}

    int begin()
    {
      if(!_model.blackboxEnabled()) return 0;

      _serial = Hardware::getSerialPort(_model.config.serial, SERIAL_FUNCTION_BLACKBOX);
      if(!_serial) return 0;

      serialWriteInit(_serial);

      systemConfigMutable()->activeRateProfile = 0;
      systemConfigMutable()->debug_mode = debugMode = DEBUG_GYRO;

      controlRateConfig_t *rp = controlRateProfilesMutable(systemConfig()->activeRateProfile);
      rp->rcRate8 = _model.config.inputRate[ROLL];
      rp->rcExpo8 = _model.config.inputExpo[ROLL];
      rp->rcYawRate8 = _model.config.inputRate[YAW];
      rp->rcYawExpo8 = _model.config.inputExpo[YAW];
      rp->thrMid8 = 50;
      rp->thrExpo8 = 0;
      rp->dynThrPID = 0;
      rp->tpa_breakpoint = 1650;
      rp->rates[ROLL] = _model.config.inputSuperRate[ROLL];
      rp->rates[PITCH] = _model.config.inputSuperRate[PITCH];
      rp->rates[YAW] = _model.config.inputSuperRate[YAW];

      pidProfile_s * cp = currentPidProfile = &_pidProfile;
      for(size_t i = 0; i < PID_ITEM_COUNT; i++)
      {
        cp->pid[i].P = _model.config.pid[i].P;
        cp->pid[i].I = _model.config.pid[i].I;
        cp->pid[i].D = _model.config.pid[i].D;
      }
      cp->pidAtMinThrottle = 1;
      cp->dterm_filter_type = _model.config.dtermFilterType;
      cp->dterm_lpf_hz = _model.config.dtermFilterCutFreq;
      cp->yaw_lpf_hz = _model.config.gyroFilterCutFreq;
      cp->itermWindupPointPercent = _model.config.itermWindupPointPercent;
      cp->dtermSetpointWeight = _model.config.dtermSetpointWeight;

      rcControlsConfigMutable()->deadband = _model.config.inputDeadband;
      rcControlsConfigMutable()->yaw_deadband = _model.config.inputDeadband;

      gyroConfigMutable()->gyro_lpf = _model.config.gyroDlpf;
      gyroConfigMutable()->gyro_soft_lpf_type = _model.config.gyroFilterType;
      gyroConfigMutable()->gyro_soft_lpf_hz = _model.config.gyroFilterCutFreq;

      accelerometerConfigMutable()->acc_lpf_hz = _model.config.accelFilterCutFreq;
      accelerometerConfigMutable()->acc_hardware = _model.config.accelDev;
      barometerConfigMutable()->baro_hardware = _model.config.baroDev;
      compassConfigMutable()->mag_hardware = _model.config.magDev;

      motorConfigMutable()->dev.useUnsyncedPwm = _model.config.outputAsync;
      motorConfigMutable()->dev.motorPwmProtocol = _model.config.outputProtocol;
      motorConfigMutable()->dev.motorPwmRate = _model.config.outputRate;

      blackboxConfigMutable()->p_denom = _model.config.blackboxPdenom;
      blackboxConfigMutable()->device = _model.config.blackboxDev;

      targetPidLooptime = gyro.targetLooptime = _model.state.gyroSampleInterval;

      motorConfigMutable()->minthrottle = motorOutputLow = _model.config.outputMin[0];
      motorConfigMutable()->maxthrottle = motorOutputHigh = _model.config.outputMax[0];

      gyroConfigMutable()->gyro_sync_denom = _model.config.gyroSync;
      pidConfigMutable()->pid_process_denom = _model.config.loopSync;

      featureConfigMutable()->enabledFeatures = _model.config.featureMask;

      blackboxInit();

      return 1;
    }

    int update()
    {
      if(!_serial) return 0;
      _model.state.stats.start(COUNTER_BLACKBOX);
      updateArmed();
      updateMode();
      updateData();
      blackboxUpdate(_model.state.loopTimestamp);
      _model.state.stats.end(COUNTER_BLACKBOX);
      return 1;
    }

  private:
    void updateData()
    {
      for(size_t i = 0; i < 3; i++)
      {
        gyro.gyroADCf[i] = degrees(_model.state.gyro[i]);
        acc.accSmooth[i] = _model.state.accel[i] * 2048.f;
        axisPID_P[i] = _model.state.innerPid[i].pTerm * 1000.f;
        axisPID_I[i] = _model.state.innerPid[i].iTerm * 1000.f;
        axisPID_D[i] = _model.state.innerPid[i].dTerm * 1000.f;
        rcCommand[i] = _model.state.input[i] * 500;
        switch(debugMode)
        {
          case DEBUG_GYRO:
            debug[i] = lrintf(degrees(_model.state.gyroScaled[i]));
            break;
          case DEBUG_ANGLERATE:
            debug[i] = lrintf(degrees(_model.state.desiredRate[i]));
            break;
          default:
            debug[i] = 0;
        }
      }
      rcCommand[AXIS_THRUST] = Math::map(_model.state.input[AXIS_THRUST], -1.f, 1.f, 1000, 2000);
      for(size_t i = 0; i < 4; i++)
      {
        motor[i] = Math::bound((int)_model.state.outputUs[i], 1000, 2000);
      }
    }

    void updateArmed()
    {
      static uint32_t beep = 0;
      if(beep != 0 && beep < _model.state.loopTimestamp)
      {
        setArmingBeepTimeMicros(_model.state.loopTimestamp);
        beep = 0;
      }

      bool armed =_model.isActive(MODE_ARMED);
      if(armed == ARMING_FLAG(ARMED)) return;
      if(armed)
      {
        ENABLE_ARMING_FLAG(ARMED);
        beep = _model.state.loopTimestamp + 200000; // delay arming beep event ~20ms
      }
      else
      {
        DISABLE_ARMING_FLAG(ARMED);
        setArmingBeepTimeMicros(micros());
        blackboxFinish();
      }
    }

    void updateMode()
    {
      if(_model.isActive(MODE_ANGLE)) bitArraySet(&rcModeActivationMask, BOXANGLE);
      else bitArrayClr(&rcModeActivationMask, BOXANGLE);

      if(_model.isActive(MODE_AIRMODE)) bitArraySet(&rcModeActivationMask, BOXAIRMODE);
      else bitArrayClr(&rcModeActivationMask, BOXAIRMODE);
    }

    Model& _model;
    pidProfile_s _pidProfile;
    HardwareSerial * _serial;
};

}
#endif
