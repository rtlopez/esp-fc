#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include "Rc/Crsf.h"
#include "TelemetryManager.h"

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

    int begin(Device::SerialDevice * serial, TelemetryManager * telemetry);
    virtual InputStatus update() override;
    virtual uint16_t get(uint8_t i) const override;
    virtual void get(uint16_t * data, size_t len) const override;
    virtual size_t getChannelCount() const override;
    virtual bool needAverage() const override;

    void print(char c) const;
    void parse(Rc::CrsfMessage& frame, int d);

  private:
    void reset();
    void apply(const Rc::CrsfMessage& msg);
    void applyLinkStats(const Rc::CrsfMessage& msg);
    void applyChannels(const Rc::CrsfMessage& msg);
    void applyMspReq(const Rc::CrsfMessage& msg);

    static constexpr size_t CHANNELS = 16;
    static constexpr size_t TELEMETRY_INTERVAL = 20000;

    Device::SerialDevice * _serial;
    TelemetryManager * _telemetry;
    CrsfState _state;
    uint8_t _idx;
    bool _new_data;
    Rc::CrsfMessage _frame;
    uint16_t _channels[CHANNELS];
    uint32_t _telemetry_next;
};

}

}
