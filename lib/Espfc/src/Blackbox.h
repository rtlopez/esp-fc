#ifndef _ESPFC_BLACKBOX_H_
#define _ESPFC_BLACKBOX_H_
#include "Model.h"

extern "C" {
#include "blackbox.h"

static Stream * blackboxSerial = NULL;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
  UNUSED(instance);
  if(blackboxSerial) blackboxSerial->write(ch);
}

void serialWriteInit(Stream& serial)
{
  blackboxSerial = &serial;
}

}

namespace Espfc {

class Blackbox
{
  public:
    Blackbox(Model& model, Stream& serial): _model(model), _serial(serial) {}
    void begin()
    {
      serialWriteInit(_serial);

      currentPidProfile = &_pidProfile;
      blackboxConfigMutable()->p_denom = 32;
      blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
      gyro.targetLooptime = _model.state.gyroSampleInterval * 1000;
      targetPidLooptime = gyro.targetLooptime;

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

      blackboxUpdate(micros());
    }

  private:
    Model& _model;
    Stream& _serial;
    pidProfile_s _pidProfile;
};

}
#endif
