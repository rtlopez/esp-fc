#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Rc/Crsf.h"

// https://github.com/CapnBry/CRServoF/blob/master/lib/CrsfSerial/crsf_protocol.h
// https://github.com/AlessioMorale/crsf_parser/tree/master
// https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c

namespace Espfc {

namespace Device {

class InputCRSF: public InputDevice
{
  public:
    enum CrsfState {
      CRSF_ADDR,
      CRSF_SIZE,
      CRSF_TYPE,
      CRSF_DATA,
      CRSF_CRC
    };

    InputCRSF();

    int begin(Device::SerialDevice * serial);
    virtual InputStatus update() override;
    virtual uint16_t get(uint8_t i) const override;
    virtual void get(uint16_t * data, size_t len) const override;
    virtual size_t getChannelCount() const override;
    virtual bool needAverage() const override;

    void print(char c) const;
    void parse(Rc::CrsfFrame& frame, int d);

  private:
    void reset();
    void apply(const Rc::CrsfFrame& frame);
    void applyLinkStats(const Rc::CrsfFrame f);
    void applyChannels(const Rc::CrsfFrame f);

    static const size_t CHANNELS = 16;

    Device::SerialDevice * _serial;
    CrsfState _state;
    uint8_t _idx;
    bool _new_data;
    Rc::CrsfFrame _frame;
    uint16_t _channels[CHANNELS];
};

}

}
