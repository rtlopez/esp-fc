#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_
#include "Model.h"

extern "C" {
#include "blackbox.h"

static HardwareSerial * blackboxSerial = NULL;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
  UNUSED(instance);
  if(blackboxSerial) blackboxSerial->write(ch);
}

void serialWriteInit(HardwareSerial& serial)
{
  blackboxSerial = &serial;
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
    Blackbox(Model& model, HardwareSerial& serial): _model(model), _serial(serial) {}
    void begin()
    {
      serialWriteInit(_serial);

      systemConfigMutable()->activeRateProfile = 0;
      systemConfigMutable()->debug_mode = 8;

      controlRateConfig_t *rp = controlRateProfilesMutable(systemConfig()->activeRateProfile);
      rp->rcRate8 = 57;
      rp->rcExpo8 = 0;
      rp->rcYawRate8 = 130;
      rp->rcYawExpo8 = 0;
      rp->thrMid8 = 50;
      rp->thrExpo8 = 0;
      rp->dynThrPID = 10;
      rp->tpa_breakpoint = 1650;
      rp->rates[ROLL] = 84;
      rp->rates[PITCH] = 84;
      rp->rates[YAW] = 50;

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
      cp->dterm_lpf_hz = 90;
      cp->yaw_lpf_hz = 90;
      cp->itermWindupPointPercent = 50;

      gyroConfigMutable()->gyro_lpf = 0;
      gyroConfigMutable()->gyro_soft_lpf_type = 0;
      gyroConfigMutable()->gyro_soft_lpf_hz = 90;

      accelerometerConfigMutable()->acc_lpf_hz = 10;

      motorConfigMutable()->dev.useUnsyncedPwm = 1;
      motorConfigMutable()->dev.motorPwmProtocol = 0;
      motorConfigMutable()->dev.motorPwmRate = _model.config.pwmRate;


      blackboxConfigMutable()->p_denom = 32;
      blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;

      gyro.targetLooptime = _model.state.gyroSampleInterval * 1000;
      targetPidLooptime = gyro.targetLooptime;

      motorConfigMutable()->minthrottle = motorOutputLow = 1060;
      motorConfigMutable()->maxthrottle = motorOutputHigh = 2000;

      gyroConfigMutable()->gyro_sync_denom = 1;
      pidConfigMutable()->pid_process_denom = 1;

      featureConfigMutable()->enabledFeatures = 9243034;

      blackboxInit();
    }

    void  update()
    {
      if(_model.state.armed)
      {
        setArmingBeepTimeMicros(micros());
        ENABLE_ARMING_FLAG(ARMED);
      }
      else
      {
        setArmingBeepTimeMicros(micros());
        DISABLE_ARMING_FLAG(ARMED);
      }
      if(_model.state.newGyroData)
      {
        blackboxUpdate(micros());
      }
    }

  private:
    Model& _model;
    HardwareSerial& _serial;
    pidProfile_s _pidProfile;
};

}
#endif
