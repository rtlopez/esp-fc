#include "Sensor/GpsSensor.hpp"
#include <GpsProtocol.hpp>
#include <Arduino.h>

namespace Espfc::Sensor
{

static constexpr std::array<int, 6> BAUDS{
  115200, 9600, 230400, 57600, 38400, 19200,
};

static constexpr std::array<uint16_t, 6> NMEA_MSG_OFF{
  Gps::NMEA_MSG_GGA, Gps::NMEA_MSG_GLL, Gps::NMEA_MSG_GSA,
  Gps::NMEA_MSG_GSV, Gps::NMEA_MSG_RMC, Gps::NMEA_MSG_VTG,
};

static constexpr std::array<uint16_t, 2> UBX_MSG_ON{
  Gps::UBX_NAV_PVT,
  Gps::UBX_NAV_SAT,
  //Gps::UBX_NAV_SOL,
  //Gps::UBX_HNR_PVT,
};

GpsSensor::GpsSensor(Model& model): _model(model) {}

int GpsSensor::begin(Device::SerialDevice* port)
{
  _port = port;
  _timer.setRate(50);

  _model.logger.info().log(F("GPS")).logln(1);

  setState(WAIT, WAIT, DETECT_BAUD);
  _timeout = micros() + 3000000;


  return 1;
}

int GpsSensor::update()
{
  if(!_port) return 0;

  if(!_timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_GPS_READ);

  bool updated = false;
  uint8_t buff[32];
  size_t read = 0;
  while ((read = _port->readMany(buff, sizeof(buff))))
  {
    for (size_t i = 0; i < read; i++)
    {
      updated |= processUbx(buff[i]);
      processNmea(buff[i]);
    }
  }

  if (!updated) handle();

  return 1;
}

bool GpsSensor::processUbx(uint8_t c)
{
  _ubxParser.parse(c, _ubxMsg);
  if (!_ubxMsg.isReady()) return false;

  handle();
  _ubxMsg = Gps::UbxMessage();

  return true;
}

void GpsSensor::processNmea(uint8_t c)
{
  _nmeaParser.parse(c, _nmeaMsg);
  if (!_nmeaMsg.isReady()) return;

  char buff[64];
  size_t len = std::min(_nmeaMsg.length, sizeof(buff) - 1);
  std::copy(_nmeaMsg.payload, _nmeaMsg.payload + len, buff);
  buff[len] = 0;
  D(buff);

  _nmeaMsg = Gps::NmeaMessage();
}

void GpsSensor::handle()
{
  switch (_state)
  {
    case DETECT_BAUD:
      setBaud(BAUDS[_counter]);
      _counter++;
      if (_counter < BAUDS.size())
      {
        send(Gps::UbxMonVer{}, _state, DETECT_BAUD);
      }
      else
      {
        send(Gps::UbxMonVer{}, ERROR, ERROR);
        _counter = 0;
      }
      break;

    case DISABLE_NMEA:
    {
      const Gps::UbxCfgMsg8 m{
        .msgId = NMEA_MSG_OFF[_counter],
        .channels = {0, 0, 0, 0, 0, 0},
      };
      _counter++;
      if (_counter < NMEA_MSG_OFF.size())
      {
        send(m, _state);
      }
      else
      {
        _counter = 0;
        send(m, SET_BAUD);
        _model.logger.info().log(F("GPS NMEA")).logln(0);
      }
    }
    break;

    case SET_BAUD:
    {
      const Gps::UbxCfgPrt20 m{
        .portId = 1,
        .resered1 = 0,
        .txReady = 0,
        .mode = 0x08c0,     // 8N1
        .baudRate = 115200, // baud
        .inProtoMask = 0x07,
        .outProtoMask = 0x07,
        .flags = 0,
        .resered2 = 0,
      };
      // if(!_counter++) {
      send(m, ENABLE_UBX);
      //} else {
      //  send(m, ENABLE_UBX, ERROR);
      _counter = 0;
      //}
      delay(35); // wait until transmission complete at 9600bps
      setBaud(115200);
    }
    break;

    case ENABLE_UBX:
    {
      const Gps::UbxCfgMsg8 m{
        .msgId = UBX_MSG_ON[_counter],
        .channels = {0, 1, 0, 0, 0, 0},
      };
      _counter++;
      if (_counter < UBX_MSG_ON.size())
      {
        send(m, _state);
      }
      else
      {
        send(m, ENABLE_SBAS);
        _counter = 0;
        _timeout = micros() + 10 * TIMEOUT;
        _model.logger.info().log(F("GPS UBX")).logln(1);
      }
    }
    break;

    case ENABLE_SBAS:
    {
      if (_model.state.gps.support.sbas)
      {
        const Gps::UbxCfgSbas8 m{
          .mode = 1,
          .usage = 1,
          .maxSbas = 3,
          .scanmode2 = 0,
          .scanmode1 = 0,
        };
        send(m, SET_RATE);
      }
      else
      {
        setState(SET_RATE);
      }
      _model.logger.info().log(F("GPS SBAS")).logln(_model.state.gps.support.sbas);
    }
    break;

    case SET_RATE:
    {
      const uint16_t mRate = 200;
      const uint16_t nRate = 10;
      const Gps::UbxCfgRate6 m{
        .measRate = mRate,
        .navRate = nRate,
        .timeRef = 0, // utc
      };
      send(m, RECEIVE);
      _model.logger.info().log(F("GPS RATE")).log(mRate).logln(nRate);
    }
    break;

    case ERROR:
      if (_counter == 0)
      {
        _model.logger.err().logln(F("GPS ERROR"));
        _counter++;
      }
      handleError();
      break;

    case RECEIVE:
      _model.state.gps.present = true;
    case WAIT:
    default:
      if (_ubxMsg.isReady())
      {
        if (_ubxMsg.isAck())
        {
          _state = _ackState;
        }
        else if (_ubxMsg.isNak())
        {
          _state = ERROR;
        }
        else if (_ubxMsg.isResponse(Gps::UbxMonVer::ID))
        {
          handleVersion();
          _state = DISABLE_NMEA;
          _counter = 0;
        }
        else if (_ubxMsg.isResponse(Gps::UbxNavPvt92::ID))
        {
          handleNavPvt();
        }
        else if (_ubxMsg.isResponse(Gps::UbxNavSat::ID))
        {
          handleNavSat();
        }
      }
      else if (_state == WAIT && micros() > _timeout)
      {
        // timeout
        _state = _timeoutState;
        _model.state.gps.present = false;
      }
      break;
  }
}

void GpsSensor::setBaud(int baud)
{
  _port->updateBadRate(baud);
  _model.logger.info().log(F("GPS BAUD")).logln(BAUDS[_counter]);
}

void GpsSensor::setState(State state, State ackState, State timeoutState)
{
  setState(state);
  _ackState = ackState;
  _timeoutState = timeoutState;
}

void GpsSensor::setState(State state)
{
  _state = state;
  _timeout = micros() + TIMEOUT;
}

void GpsSensor::handleError() const
{
  _model.state.gps.present = false;
}

void GpsSensor::handleNavPvt() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();
  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.height;

  _model.state.gps.velocity.raw.north = m.velN;
  _model.state.gps.velocity.raw.east = m.velE;
  _model.state.gps.velocity.raw.down = m.velD;
  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed;
  _model.state.gps.velocity.raw.heading = m.headMot;

  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;
  _model.state.gps.accuracy.pDop = m.pDOP;

  //_hal->printf("\n");
  //_hal->printf("Dat: %d-%d-%d %d:%d:%d %d\n", m.year, m.month, m.day, m.hour, m.min, m.sec, m.iTow);
  //_hal->printf("Pos: %.7f %.7f %.7f %d %d %d\n", 1e-7f * m.lat, 1e-7f * m.lon, 1e-3f * m.height, m.lat, m.lon, m.height);
  //_hal->printf("Vel: %d %d %d %d %d\n", m.velN, m.velE, m.velD, m.gSpeed, m.headMot);
  //_hal->printf("Fix: %d %d\n", m.numSV, m.fixType);
  //_hal->printf("Acu: %d %d %d %d %d\n", m.hAcc, m.vAcc, m.sAcc, m.headAcc, m.pDOP);
}

void GpsSensor::handleNavSat() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavSat>();
  for (uint8_t i = 0; i < SAT_MAX; i++)
  {
    if(i < m.numSvs)
    {
      _model.state.gps.svinfo[i].id = m.sats[i].svId;
      _model.state.gps.svinfo[i].gnssId = m.sats[i].gnssId;
      _model.state.gps.svinfo[i].cno = m.sats[i].cno;
      _model.state.gps.svinfo[i].quality = m.sats[i].flags.qualityInd;
    }
    else
    {
      _model.state.gps.svinfo[i] = GpsSatelite{};
    }
  }
}

void GpsSensor::handleVersion() const
{
  const char *payload = (const char *)_ubxMsg.payload;

  _model.logger.info().log(F("GPS VER")).logln(payload);
  _model.logger.info().log(F("GPS VER")).logln(payload + 30);

  if (std::strcmp(payload + 30, "00080000") == 0)
  {
    _model.state.gps.support.version = GPS_M8;
  }
  else if (std::strcmp(payload + 30, "00090000") == 0)
  {
    _model.state.gps.support.version = GPS_M9;
  }
  else if (std::strcmp(payload + 30, "00190000") == 0)
  {
    _model.state.gps.support.version = GPS_F9;
  }
  if (_ubxMsg.length >= 70)
  {
    checkSupport(payload + 40);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 40);
  }
  if (_ubxMsg.length >= 100)
  {
    checkSupport(payload + 70);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 70);
  }
  if (_ubxMsg.length >= 130)
  {
    checkSupport(payload + 100);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 100);
  }
  if (_ubxMsg.length >= 160)
  {
    checkSupport(payload + 130);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 130);
  }
}

void GpsSensor::checkSupport(const char *payload) const
{
  if (std::strstr(payload, "SBAS") != nullptr)
  {
    _model.state.gps.support.sbas = true;
  }
  if (std::strstr(payload, "GLO") != nullptr)
  {
    _model.state.gps.support.glonass = true;
  }
  if (std::strstr(payload, "GAL") != nullptr)
  {
    _model.state.gps.support.galileo = true;
  }
  if (std::strstr(payload, "BDS") != nullptr)
  {
    _model.state.gps.support.beidou = true;
  }
}

}
