#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Rc/Crsf.h"
#include "Utils/Math.hpp"
#include <algorithm>

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
  CRSF_TELEMETRY_STATE_BARO,
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
        // If no GPS don't waste time sending empty messages
        _current = _model.gpsActive() ? CRSF_TELEMETRY_STATE_GPS : (_model.baroActive() ? CRSF_TELEMETRY_STATE_BARO : CRSF_TELEMETRY_STATE_HB);
        break;
      case CRSF_TELEMETRY_STATE_GPS:
        gps(f);
        send(f, s);
        // If no barometer don't send the message
        _current = _model.baroActive() ? CRSF_TELEMETRY_STATE_BARO : CRSF_TELEMETRY_STATE_HB;
        break;
      case CRSF_TELEMETRY_STATE_BARO:
        vario(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_HB;
        break;
      default:    // In case of an invalid state, send heartbeat and continue loop
        heartbeat(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_ATTI;
        break;
    }

    return 1;
  }

  int sendMsp(Device::SerialDevice& s, Connect::MspResponse resp, uint8_t origin)
  {
    size_t size = resp.serialize(_buff, sizeof(_buff));
    const uint8_t* beg = _buff + 3;        // skip msp header
    const uint8_t* end = _buff + size - 1; // skip crc
    uint8_t version = resp.version == Connect::MSP_V1 ? 1 : 2;
    size_t iter = 0;
    Rc::CrsfMessage frame;
    do
    {
      beg = Rc::Crsf::encodeMspData(frame, origin, version, _seq++, !iter, beg, end);
      send(frame, s);
      iter++;
    } while(beg != end && iter < 4);

    return iter;
  }

  void send(const Rc::CrsfMessage& msg, Device::SerialDevice& s) const
  {
    s.write((uint8_t*)&msg, msg.size + 2);
  }

  int16_t toAngle(float angle) const
  {
    if(angle < -Utils::pi()) angle += Utils::twoPi();
    if(angle >  Utils::pi()) angle -= Utils::twoPi();
    return lrintf(angle * 10000);
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

  void gps(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_GPS);

    msg.writeU32(Utils::toBigEndian32(_model.state.gps.location.raw.lat)); // deg * 1e7
    msg.writeU32(Utils::toBigEndian32(_model.state.gps.location.raw.lon)); // deg * 1e7
    msg.writeU16(Utils::toBigEndian16((_model.state.gps.velocity.raw.groundSpeed * 36 + 500) / 1000)); // in km/h * 10
    msg.writeU16(Utils::toBigEndian16((_model.state.gps.velocity.raw.heading + 500) / 1000)); // deg * 10
    uint16_t altitude = std::clamp((_model.state.gps.location.raw.height + 500) / 1000, (int32_t)-900, (int32_t)5000) + 1000; // m
    msg.writeU16(Utils::toBigEndian16(altitude));
    msg.writeU8(_model.state.gps.numSats);

    msg.finalize();
  }

  void vario(Rc::CrsfMessage& msg) const
  {
    // https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_BARO_ALTITUDE
    msg.prepare(Rc::CRSF_FRAMETYPE_BARO_ALTITUDE);

    // Send barometer data
    msg.writeU16(Utils::toBigEndian16((_model.state.baro.altitude * 10.0f) + 10000 )); // (dm + 10000) or (m + 0 | 0x8000)
    msg.writeU16(Utils::toBigEndian16(_model.state.baro.vario * 100.0f)); // cm

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
  uint8_t _buff[256] = {0};
  uint8_t _seq = 0;
};

}

}
