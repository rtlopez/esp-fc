#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Rc/Crsf.h"
#include "Utils/Math.hpp"

// https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_MSP_REQ - not correct
// https://github.com/betaflight/betaflight/blob/2525be9a3369fa666d8ce1485ec5ad344326b085/src/main/telemetry/crsf.c#L664
// https://github.com/betaflight/betaflight/blob/2525be9a3369fa666d8ce1485ec5ad344326b085/src/main/telemetry/msp_shared.c#L46

namespace Espfc {

namespace Telemetry {

enum TelemetryState {
  CRSF_TELEMETRY_STATE_ATTI,
  CRSF_TELEMETRY_STATE_BAT,
  CRSF_TELEMETRY_STATE_FM,
  CRSF_TELEMETRY_STATE_GPS,
  CRSF_TELEMETRY_STATE_VARIO,
  CRSF_TELEMETRY_STATE_HB,
};

class TelemetryCRSF
{
public:
  TelemetryCRSF(Model& model): _model(model) {}

  int begin()
  {
    return 1;
  }

  int process(Device::SerialDevice& s) const
  {
    Rc::CrsfMessage f;
    switch(_current)
    {
      case CRSF_TELEMETRY_STATE_ATTI:
        attitude(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_BAT;
        break;
      case CRSF_TELEMETRY_STATE_BAT:
        battery(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_FM;
        break;
      case CRSF_TELEMETRY_STATE_FM:
        flightMode(f);
        send(f, s);
        //_current = CRSF_TELEMETRY_STATE_GPS;
        _current = CRSF_TELEMETRY_STATE_VARIO;
        break;
      case CRSF_TELEMETRY_STATE_GPS:
        //gps(f);
        //send(f, s);
        _current = CRSF_TELEMETRY_STATE_VARIO;
        break;
      case CRSF_TELEMETRY_STATE_VARIO:
        vario(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_HB;
        break;
      case CRSF_TELEMETRY_STATE_HB:
        heartbeat(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_ATTI;
        break;
    }

    return 1;
  }

  int sendMsp(Device::SerialDevice& s, Connect::MspResponse r, uint8_t origin) const
  {
    Rc::CrsfMessage msg;

    Rc::Crsf::encodeMsp(msg, r, origin);

    send(msg, s);

    return 1;
  }

  void send(const Rc::CrsfMessage& msg, Device::SerialDevice& s) const
  {
    s.write((uint8_t*)&msg, msg.size + 2);
  }

  int16_t toAngle(float angle) const
  {
    if(angle < -Utils::pi()) angle += Utils::twoPi();
    if(angle >  Utils::pi()) angle -= Utils::twoPi();
    return lrintf(angle * 1000);
  }

  void attitude(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_ATTITUDE);

    int16_t r = toAngle(_model.state.attitude.euler.x);
    int16_t p = toAngle(_model.state.attitude.euler.y);
    int16_t y = toAngle(_model.state.attitude.euler.z);

    msg.writeU16(Utils::toBigEndian16(r));
    msg.writeU16(Utils::toBigEndian16(p));
    msg.writeU16(Utils::toBigEndian16(y));

    msg.finalize();
  }

  void battery(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_BATTERY_SENSOR);

    uint16_t voltage = Utils::clamp(lrintf(_model.state.battery.voltage * 10.0f), 0l, 32000l);
    uint16_t current = Utils::clamp(lrintf(_model.state.battery.current * 10.0f), 0l, 32000l);
    uint32_t mahDrawn = 0;
    uint8_t remainPerc = lrintf(_model.state.battery.percentage);

    msg.writeU16(Utils::toBigEndian16(voltage));
    msg.writeU16(Utils::toBigEndian16(current));
    msg.writeU8(mahDrawn >> 16);
    msg.writeU8(mahDrawn >> 8);
    msg.writeU8(mahDrawn);
    msg.writeU8(remainPerc);

    msg.finalize();
  }

  void flightMode(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_FLIGHT_MODE);

    if(_model.armingDisabled()) msg.writeString("!DIS");
    if(_model.isModeActive(MODE_FAILSAFE)) msg.writeString("!FS,");
    if(_model.isModeActive(MODE_ARMED)) msg.writeString("ARM,");
    if(_model.isModeActive(MODE_AIRMODE)) msg.writeString("AIR,");
    if(_model.isModeActive(MODE_ANGLE)) msg.writeString("STAB,");
    msg.writeU8(0);

    msg.finalize();
  }

  void vario(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_VARIO_SENSOR);

    msg.writeU16(Utils::toBigEndian16(0));

    msg.finalize();
  }

  void heartbeat(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_HEARTBEAT);

    msg.writeU16(Utils::toBigEndian16(Rc::CRSF_ADDRESS_FLIGHT_CONTROLLER));

    msg.finalize();
  }

private:
  Model& _model;
  mutable TelemetryState _current;
};

}

}
