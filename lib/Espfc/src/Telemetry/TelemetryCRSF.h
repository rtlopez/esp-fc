#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Rc/Crsf.h"
#include "Math/Utils.h"

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
    Rc::CrsfFrame f;
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
        _current = CRSF_TELEMETRY_STATE_GPS;
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

  int sendMsp(Device::SerialDevice& s, Msp::MspResponse r) const
  {

    return 1;
  }

  void send(const Rc::CrsfFrame& f, Device::SerialDevice& s) const
  {
    s.write(f.data, f.message.size + 2);
  }

  int16_t toAngle(float angle) const
  {
    if(angle < -Math::pi()) angle += Math::twoPi();
    if(angle >  Math::pi()) angle -= Math::twoPi();
    return lrintf(angle * 1000);
  }

  void attitude(Rc::CrsfFrame& f) const
  {
    prepare(f, Rc::CRSF_FRAMETYPE_ATTITUDE);

    int16_t r = toAngle(_model.state.angle.x);
    int16_t p = toAngle(_model.state.angle.y);
    int16_t y = toAngle(_model.state.angle.z);

    f.message.writeU16(Math::toBigEndian16(r));
    f.message.writeU16(Math::toBigEndian16(p));
    f.message.writeU16(Math::toBigEndian16(y));

    finalize(f);
  }

  void battery(Rc::CrsfFrame& f) const
  {
    prepare(f, Rc::CRSF_FRAMETYPE_BATTERY_SENSOR);

    uint16_t voltage = Math::clamp(lrintf(_model.state.battery.voltage * 10.0f), 0l, 32000l);
    uint16_t current = Math::clamp(lrintf(_model.state.battery.current * 10.0f), 0l, 32000l);
    uint32_t mahDrawn = 0;
    uint8_t remainPerc = 100;

    f.message.writeU16(Math::toBigEndian16(voltage));
    f.message.writeU16(Math::toBigEndian16(current));
    f.message.writeU8(mahDrawn >> 16);
    f.message.writeU8(mahDrawn >> 8);
    f.message.writeU8(mahDrawn);
    f.message.writeU8(remainPerc);

    finalize(f);
  }

  void flightMode(Rc::CrsfFrame& f) const
  {
    prepare(f, Rc::CRSF_FRAMETYPE_FLIGHT_MODE);

    if(_model.armingDisabled()) f.message.writeString("!DIS");
    if(_model.isModeActive(MODE_FAILSAFE)) f.message.writeString("!FS,");
    if(_model.isModeActive(MODE_ARMED)) f.message.writeString("ARM,");
    if(_model.isModeActive(MODE_AIRMODE)) f.message.writeString("AIR,");
    if(_model.isModeActive(MODE_ANGLE)) f.message.writeString("STAB,");
    f.message.writeU8(0);

    finalize(f);
  }

  void vario(Rc::CrsfFrame& f) const
  {
    prepare(f, Rc::CRSF_FRAMETYPE_VARIO_SENSOR);

    f.message.writeU16(Math::toBigEndian16(0));

    finalize(f);
  }

  void heartbeat(Rc::CrsfFrame& f) const
  {
    prepare(f, Rc::CRSF_FRAMETYPE_HEARTBEAT);

    f.message.writeU16(Math::toBigEndian16(Rc::CRSF_ADDRESS_FLIGHT_CONTROLLER));

    finalize(f);
  }

  void prepare(Rc::CrsfFrame& f, uint8_t type) const
  {
    f.message.addr = Rc::CRSF_SYNC_BYTE;
    f.message.type = type;
    f.message.size = 0;
  }

  void finalize(Rc::CrsfFrame& f) const
  {
    f.message.size += 2;
    f.message.writeCRC(Rc::Crsf::crc(f));
  }

private:
  Model& _model;
  mutable TelemetryState _current;
};

}

}
