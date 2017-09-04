#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_

#include "Model.h"
#include "Hardware.h"

extern "C" {
#include "blackbox.h"

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
      if(!_model.config.blackbox) return 0;

      _serial = Hardware::getSerialPort(_model.config.blackboxPort);
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
      rp->dynThrPID = 10;
      rp->tpa_breakpoint = 1650;
      rp->rates[ROLL] = _model.config.inputSuperRate[ROLL];
      rp->rates[PITCH] = _model.config.inputSuperRate[PITCH];
      rp->rates[YAW] = _model.config.inputSuperRate[YAW];

      pidProfile_s * cp = currentPidProfile = &_pidProfile;
      cp->pid[ROLL].P = 50;
      cp->pid[ROLL].I = 40;
      cp->pid[ROLL].D = 20;
      cp->pid[PITCH].P = 50;
      cp->pid[PITCH].I = 40;
      cp->pid[PITCH].D = 20;
      cp->pid[YAW].P = 50;
      cp->pid[YAW].I = 40;
      cp->pid[YAW].D = 20;
      cp->pid[PID_LEVEL].P = 50;
      cp->pid[PID_LEVEL].I = 50;
      cp->pid[PID_LEVEL].D = 90;
      cp->pidAtMinThrottle = 1;
      cp->dterm_filter_type = 1;
      cp->dterm_lpf_hz = 100;
      cp->yaw_lpf_hz = _model.config.gyroFilterCutFreq;
      cp->itermWindupPointPercent = 50;

      gyroConfigMutable()->gyro_lpf = 0;
      gyroConfigMutable()->gyro_soft_lpf_type = _model.config.gyroFilterType;
      gyroConfigMutable()->gyro_soft_lpf_hz = _model.config.gyroFilterCutFreq;

      accelerometerConfigMutable()->acc_lpf_hz = _model.config.accelFilterCutFreq;
      accelerometerConfigMutable()->acc_hardware = 3;
      barometerConfigMutable()->baro_hardware = 2;
      compassConfigMutable()->mag_hardware = 2;

      motorConfigMutable()->dev.useUnsyncedPwm = 1;
      motorConfigMutable()->dev.motorPwmProtocol = 0;
      motorConfigMutable()->dev.motorPwmRate = _model.config.pwmRate;

      blackboxConfigMutable()->p_denom = 64;
      blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;

      targetPidLooptime = gyro.targetLooptime = _model.state.gyroSampleInterval;

      motorConfigMutable()->minthrottle = motorOutputLow = _model.config.outputMin[0];
      motorConfigMutable()->maxthrottle = motorOutputHigh = _model.config.outputMax[0];

      gyroConfigMutable()->gyro_sync_denom = 1;
      pidConfigMutable()->pid_process_denom = _model.config.pidSync;

      featureConfigMutable()->enabledFeatures = FEATURE_RX_PPM | FEATURE_MOTOR_STOP | FEATURE_AIRMODE | FEATURE_ANTI_GRAVITY;

      blackboxInit();

      setArmingBeepTimeMicros(micros());

      return 1;
    }

    int update()
    {
      if(!_model.config.blackbox) return 0;
      if(!_serial) return 0;

      updateArmed();
      updateData();
      blackboxUpdate(_model.state.loopTimestamp);

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
        debug[i] = lrintf(degrees(_model.state.gyroScaled[i]));
      }
      rcCommand[AXIS_THRUST] = Math::map(_model.state.input[AXIS_THRUST], -1.f, 1.f, 1000, 2000);
      for(size_t i = 0; i < 4; i++)
      {
        motor[i] = Math::bound((int)_model.state.outputUs[i], 1000, 2000);
      }
    }

    void updateArmed()
    {
      if(_model.state.armed == ARMING_FLAG(ARMED)) return;
      if(_model.state.armed)
      {
        ENABLE_ARMING_FLAG(ARMED);
        setArmingBeepTimeMicros(micros());
      }
      else
      {
        DISABLE_ARMING_FLAG(ARMED);
        blackboxFinish();
        //setArmingBeepTimeMicros(micros());
      }
    }

    Model& _model;
    pidProfile_s _pidProfile;
    HardwareSerial * _serial;
};

}
#endif
