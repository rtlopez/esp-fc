#include "Sensor/GpsSensor.hpp"
#include <Gps.hpp>
#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <tuple>
#include "GpsSensor.hpp"

namespace Espfc::Sensor
{

static constexpr std::array<int, 6> BAUDS{
  9600, 115200, 230400, 57600, 38400, 19200,
};

static constexpr std::array<uint16_t, 6> NMEA_MSG_OFF{
  Gps::NMEA_MSG_GGA, Gps::NMEA_MSG_GLL, Gps::NMEA_MSG_GSA,
  Gps::NMEA_MSG_GSV, Gps::NMEA_MSG_RMC, Gps::NMEA_MSG_VTG,
};

static constexpr std::array<std::tuple<uint16_t, uint8_t>, 2> UBX_MSG_ON{
  std::make_tuple(Gps::UBX_NAV_PVT,  1u),
  std::make_tuple(Gps::UBX_NAV_SAT, 10u),
};

GpsSensor::GpsSensor(Model& model): _model(model) {}

int GpsSensor::begin(Device::SerialDevice* port, int baud)
{
  _port = port;
  _targetBaud = _currentBaud = baud;
  _timer.setRate(50);

  _state = DETECT_BAUD;
  _timeout = micros() + DETECT_TIMEOUT;
  _counter = 0;
  setBaud(_targetBaud);

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

  onMessage();

  handle();
  _ubxMsg = Gps::UbxMessage();

  return true;
}

void GpsSensor::processNmea(uint8_t c)
{
  _nmeaParser.parse(c, _nmeaMsg);
  if (!_nmeaMsg.isReady()) return;

  //$GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*70
  static const char * msg = "GNTXT,01,01,01,More than 100 frame errors";

  if(!_model.state.gps.frameError && std::strncmp(_nmeaMsg.payload, msg, std::strlen(msg)) == 0)
  {
    _model.state.gps.frameError = true;
    if(!_model.isModeActive(MODE_ARMED)) _model.logger.err().logln("GPS RX Frame Err");
  }

  onMessage();

  _nmeaMsg = Gps::NmeaMessage();
}

void GpsSensor::onMessage()
{
  if(_state == DETECT_BAUD)
  {
    _state = GET_VERSION;
    _model.logger.info().log(F("GPS DET")).logln(_currentBaud);
  }
}

void GpsSensor::handle()
{
  switch (_state)
  {
    case DETECT_BAUD:
      detectBaud();
      break;

    case GET_VERSION:
      readVersion();
      break;

    case CONFIGURE_BAUD:
      configureBaud();
      break;

    case DISABLE_NMEA:
      disableNmea();
      break;

    case ENABLE_UBX:
      enableUbx();      
      break;

    case ENABLE_NAV5:
      enableNav5();
      break;

    case ENABLE_SBAS:
      enableSbas();
      break;

    case DETECT_GPS_L5:
      detectGpsL5();
      break;

    case CONFIGURE_GNSS:
      configureGnss();
      break;

    case CONFIGURE_NAV_RATE:
      configureRate();
      break;

    case ERROR:
      handleError();
      break;

    case RECEIVE:
    case WAIT:
    default:
      handleReceive();
      break;
  }
}

void GpsSensor::handleReceive()
{
  if (_state == RECEIVE)
  {
    _model.state.gps.present = true;
  }

  if (_ubxMsg.isReady())
  {
    if (_ubxMsg.isAck())
    {
      _state = _ackState;
    }
    else if (_ubxMsg.isNak())
    {
      _state = _timeoutState;
      _model.logger.err().log(F("GPS NAK")).loghex(_ubxMsg.payload[0]).loghex(_ubxMsg.payload[1]).endl();
    }
    else if (_ubxMsg.isResponse(Gps::UBX_CFG_VALGET))
    {
      handleCfgValGet();
      _state = _ackState;
      _counter = 0;
    }
    else if (_ubxMsg.isResponse(Gps::UbxMonVer::ID))
    {
      handleVersion();
      _state = _ackState;
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
    _model.logger.err().logln(F("GPS TOUT"));
  }
}

void GpsSensor::detectBaud()
{
  if(micros() > _timeout)
  {
    // on timeout check next baud
    if(_counter < BAUDS.size())
    {
      setBaud(BAUDS[_counter]);
      _counter++;
    }
    else
    {
      _state = ERROR; // detection falied, give up
      // _state = DETECT_BAUD; // restart detection if all baud rates failed
      _counter = 0;
      setBaud(_targetBaud);
    }
    _timeout = micros() + DETECT_TIMEOUT;
  }
}

void GpsSensor::readVersion()
{
  send(Gps::UbxMonVer{}, CONFIGURE_BAUD); // version handled in WAIT/RECEIVE
  _timeout = micros() + 3 * TIMEOUT;
}

void GpsSensor::configureBaud()
{
  if (isLegacyProto())
  {
    send(Gps::UbxCfgPrt20{
      .portId = 1,
      .resered1 = 0,
      .txReady = 0,
      .mode = 0x08c0,     // 8N1
      .baudRate = (uint32_t)_targetBaud, // baud
      .inProtoMask = 0x07,
      .outProtoMask = 0x07,
      .flags = 0,
      .resered2 = 0,
    }, DISABLE_NMEA, DISABLE_NMEA); // we may not be able to receive ACK for this message
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_UART1_BAUDRATE, uint32_t>(_targetBaud));
    send(req, DISABLE_NMEA, DISABLE_NMEA);
  }
  delay(30); // wait until transmission complete at 9600bps in worst case
  setBaud(_targetBaud);
  delay(5);
}

void GpsSensor::disableNmea()
{
  if (isLegacyProto())
  {
    const Gps::UbxCfgMsg3 m{
      .msgId = NMEA_MSG_OFF[_counter],
      .rate = 0,
    };
    _counter++;
    if (_counter < NMEA_MSG_OFF.size())
    {
      send(m, _state);
    }
    else
    {
      _counter = 0;
      send(m, ENABLE_UBX);
      _model.logger.info().logln(F("GPS NMEA OFF"));
    }
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GGA_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GLL_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GSA_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GSV_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_RMC_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_VTG_UART1, bool>(0));
    send(req, ENABLE_UBX);
    _model.logger.info().logln(F("GPS NMEA* OFF"));
  }
}

void GpsSensor::enableUbx()
{
  if (isLegacyProto())
  {
    const Gps::UbxCfgMsg3 m{
      .msgId = std::get<0>(UBX_MSG_ON[_counter]),
      .rate = std::get<1>(UBX_MSG_ON[_counter]),
    };
    _counter++;
    if (_counter < UBX_MSG_ON.size())
    {
      send(m, _state);
    }
    else
    {
      send(m, ENABLE_NAV5);
      _counter = 0;
      _timeout = micros() + 10 * TIMEOUT;
      _model.logger.info().logln(F("GPS UBX ON"));
    }
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_UBX_NAV_PVT_UART1, uint8_t>(1));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_UBX_NAV_SAT_UART1, uint8_t>(10));
    send(req, ENABLE_NAV5); // if supported we get ACK, then go to get_version, else try legacy disable_nmea commands
    _model.logger.info().logln(F("GPS UBX* ON"));
  }
}

void GpsSensor::enableNav5()
{
  if (isLegacyProto())
  {
    send(Gps::UbxCfgNav5{
      .mask = { .value = 0xffff }, // all
      .dynModel = 8, // airborne
      .fixMode = 3,
      .fixedAlt = 0,
      .fixedAltVar = 10000,
      .minElev = 5,
      .drLimit = 0,
      .pDOP = 250,
      .tDOP = 250,
      .pAcc = 100,
      .tAcc = 300,
      .staticHoldThresh = 0,
      .dgnssTimeout = 60,
      .cnoThreshNumSVs = 0,
      .cnoThresh = 0,
      .reserved0 = {0, 0},
      .staticHoldMaxDist = 200,
      .utcStandard = 0,
      .reserved1 = {0, 0, 0, 0, 0},
    }, ENABLE_SBAS);
    _model.logger.info().logln(F("GPS NAV5"));
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_NAVSPG_DYNMODEL, uint8_t>(8)); // airborne
    send(req, ENABLE_SBAS);
    _model.logger.info().logln(F("GPS NAVSPG*"));
  }
}

void GpsSensor::enableSbas()
{
  if (_model.state.gps.support.sbas)
  {
    if (isLegacyProto())
    {
      send(Gps::UbxCfgSbas8{
        .mode = 1,
        .usage = 1,
        .maxSbas = 3,
        .scanmode2 = 0,
        .scanmode1 = 0,
      }, DETECT_GPS_L5);
      _model.logger.info().logln(F("GPS SBAS"));
    }
    else
    {
      Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
      req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
      req.write(Gps::UbxCfgValsetItem<Gps::CFG_SBAS_PRNSCANMASK, uint64_t>(0)); // all
      send(req, DETECT_GPS_L5);
      _model.logger.info().logln(F("GPS SBAS*"));
    }
  }
  else
  {
    setState(DETECT_GPS_L5);
  }
}

void GpsSensor::detectGpsL5()
{
  Gps::UbxRequest req(Gps::UBX_CFG_VALGET);
  req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
  req.write(Gps::CFG_SIGNAL_GPS_L5);
  send(req, CONFIGURE_GNSS, CONFIGURE_GNSS); // if supported we get ACK with value, else timeout and continue with GNSS configuration without L5 support
}

void GpsSensor::configureRate()
{
  uint16_t mRate = 200;
  if (_currentBaud > 100000) mRate = 100;
  if (_model.state.gps.support.version == GPS_M10 && _currentBaud > 200000) mRate = 40; // (proto<24 => >50ms)
  const uint16_t nRate = 1;
  
  if (isLegacyProto())
  {
    const Gps::UbxCfgRate6 m{
      .measRate = mRate,
      .navRate = nRate,
      .timeRef = 0, // utc
    };
    send(m, RECEIVE);
    _model.logger.info().log(F("GPS NAVRATE")).log(mRate).log(nRate).logln(1000 / mRate);
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_MEAS, uint16_t>(mRate));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_NAV, uint16_t>(nRate));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_TIMEREF, uint8_t>(0)); // utc
    send(req, RECEIVE);
    _model.logger.info().log(F("GPS NAVRATE*")).log(mRate).log(nRate).logln(1000 / mRate);
  }
}

void GpsSensor::setBaud(int baud)
{
  if(baud != _currentBaud)
  {
    _port->updateBaudRate(baud);
    _currentBaud = baud;
    _model.logger.info().log(F("GPS BAUD")).logln(baud);
  }
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

void GpsSensor::handleError()
{
  if (_counter == 0)
  {
    _model.logger.err().logln(F("GPS ERROR"));
    _counter++;
  }
  _model.state.gps.present = false;
}

void GpsSensor::configureGnss()
{
  const bool useDualBand = _model.config.gps.enableDualBand && _model.state.gps.support.gpsL5;
  bool enableGPS  = _model.config.gps.enableGPS;
  bool enableGLO  = _model.config.gps.enableGLONASS;
  bool enableGAL  = _model.config.gps.enableGalileo;
  bool enableBDS  = _model.config.gps.enableBeiDou;
  bool enableQZSS = _model.config.gps.enableQZSS;
  bool enableSBAS = _model.config.gps.enableSBAS;

  const auto& support = _model.state.gps.support;
  switch (_model.config.gps.gnssMode)
  {
    case 1:
      enableGPS = support.gps;
      enableGLO = enableGAL = enableBDS = enableQZSS = false;
      break;
    case 2:
      enableGPS = support.gps;
      enableGLO = support.glonass;
      enableGAL = enableBDS = enableQZSS = false;
      break;
    case 3:
      enableGPS = support.gps;
      enableGAL = support.galileo;
      enableGLO = enableBDS = enableQZSS = false;
      break;
    case 4:
      enableGPS = support.gps;
      enableBDS = support.beidou;
      enableGLO = enableGAL = enableQZSS = false;
      break;
    case 5:
      enableGPS = support.gps;
      enableGLO = support.glonass;
      enableGAL = support.galileo;
      enableBDS = support.beidou;
      enableQZSS = support.qzss;
      break;
  }

  size_t written = 0;
  if (isLegacyProto())
  {
    // const Gps::UbxCfgGnss7 gnss{
    //   .msgVer = 0,
    //   .numTrkChHw = 0,
    //   .numTrkChUse = 0xFF,
    //   .numConfigBlocks = 7,
    //   .blocks = {
    //     // GPS: L1C/A or L1+L5
    //     { 0x00, 0x08, 0x10, 0x00, (uint8_t)(enableGPS  ? 0x01 : 0x00), 0x00, (uint8_t)(useDualBand ? 0x20 : 0x01), 0x01 },
    //     // SBAS: L1C/A
    //     { 0x01, 0x01, 0x03, 0x00, (uint8_t)(enableSBAS ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //     // Galileo: E1 or E1+E5a
    //     { 0x02, 0x04, 0x08, 0x00, (uint8_t)(enableGAL  ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //     // BeiDou: B1I or B1I+B2a
    //     { 0x03, 0x08, 0x10, 0x00, (uint8_t)(enableBDS  ? 0x01 : 0x00), 0x00, (uint8_t)(false && useDualBand ? 0x80 : 0x01), 0x01 },
    //     // IMES: disabled
    //     { 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x01, 0x01 },
    //     // QZSS: L1C/A or L1+L5
    //     { 0x05, 0x00, 0x03, 0x00, (uint8_t)(enableQZSS ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //     // GLONASS: L1
    //     { 0x06, 0x08, 0x0E, 0x00, (uint8_t)(enableGLO  ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //   },
    // };
    // written = sizeof(gnss);
    // send(gnss, CONFIGURE_NAV_RATE);
    Gps::UbxRequest req{Gps::UBX_CFG_GNSS};
    uint8_t numBlocks =
      _model.state.gps.support.gps + _model.state.gps.support.sbas + 
      _model.state.gps.support.galileo + _model.state.gps.support.beidou +
      _model.state.gps.support.qzss + _model.state.gps.support.glonass +
      _model.state.gps.support.imes;

    written += req.write(Gps::UbxCfgGnssHeader{.msgVer = 0, .numTrkChHw = 32, .numTrkChUse = 0xff, .numConfigBlocks = numBlocks});
    if (_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 0, .resTrkCh = 8, .maxTrkCh = 16, .flagsEnable = enableGPS, .sigCfgMask = (uint8_t)(useDualBand ? 0x20 : 0x01), .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.sbas)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 1, .resTrkCh = 1, .maxTrkCh = 3, .flagsEnable = enableSBAS, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.galileo)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 2, .resTrkCh = 4, .maxTrkCh = 8, .flagsEnable = enableGAL, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.beidou)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 3, .resTrkCh = 8, .maxTrkCh = 16, .flagsEnable = enableBDS, .sigCfgMask = (uint8_t)(false ? 0x80 : 0x01), .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.imes)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 4, .resTrkCh = 0, .maxTrkCh = 8, .flagsEnable = 0, .sigCfgMask = 0x01, .flagsHigh = 0x03});
    }
    if (_model.state.gps.support.qzss)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 5, .resTrkCh = 0, .maxTrkCh = 3, .flagsEnable = enableQZSS, .sigCfgMask = 0x01, .flagsHigh = 0x05});
    }
    if (_model.state.gps.support.glonass)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 6, .resTrkCh = 8, .maxTrkCh = 14, .flagsEnable = enableGLO, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    send(req, CONFIGURE_NAV_RATE);
  }
  else
  {
    Gps::UbxRequest req{Gps::UBX_CFG_VALSET};
    written += req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers  = 0x01 }); // RAM only
    if (_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GPS_ENA, bool>(enableGPS));
    }
    if (_model.state.gps.support.sbas)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_SBAS_ENA, bool>(enableSBAS));
    }
    if (_model.state.gps.support.galileo)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GAL_ENA, bool>(enableGAL));
    }
    if (_model.state.gps.support.qzss)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_QZSS_ENA, bool>(enableQZSS));
    }
    if (_model.state.gps.support.glonass)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GLO_ENA, bool>(enableGLO));
    }
    if (_model.state.gps.support.beidou)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_BDS_ENA, bool>(enableBDS));
    }
    if (useDualBand &&_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GPS_L5, bool>(useDualBand));
    }
    // there is no guarantion that gps and baidou are supported together.
    // if (useDualBand && _model.state.gps.support.beidou)
    // {
    //   written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_BDS_B2A, bool>(useDualBand));
    // }
    send(req, CONFIGURE_NAV_RATE);
  }

  _model.logger.info().log(F("GPS GNSS"));
  if (isLegacyProto()) _model.logger.log(F("LEGACY"));
  if (enableGPS) _model.logger.log(F("GPS"));
  if (useDualBand) _model.logger.log(F("L1+L5"));
  if (enableGLO) _model.logger.log(F("GLO"));
  if (enableGAL) _model.logger.log(F("GAL"));
  if (enableBDS) _model.logger.log(F("BDS"));
  if (enableSBAS) _model.logger.log(F("SBAS"));
  if (enableQZSS) _model.logger.log(F("QZSS"));
  _model.logger.logln(written);
}

void GpsSensor::calculateHomeVector() const
{
  if (!_model.state.gps.isHomeValid())
  {
    _model.state.gps.distanceToHome = 0;
    _model.state.gps.directionToHome = 0;
    return;
  }

  const int32_t lat1 = _model.state.gps.location.home.lat;
  const int32_t lon1 = _model.state.gps.location.home.lon;
  const int32_t lat2 = _model.state.gps.location.raw.lat;
  const int32_t lon2 = _model.state.gps.location.raw.lon;

  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(lat1, lon1, lat2, lon2);

  _model.state.gps.distanceToHome = distance;
  _model.state.gps.directionToHome = bearing;
}

void GpsSensor::handleCfgValGet() const
{
  const uint32_t key = *(reinterpret_cast<const uint32_t*>(_ubxMsg.payload) + sizeof(Gps::UbxCfgValsetHeader));
  if (key == Gps::CFG_SIGNAL_GPS_L5)
  {
    _model.state.gps.support.gpsL5 = true;
    _model.logger.info().logln(F("GPS DET L5"));
  }
}

void GpsSensor::handleNavPvt() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();

  _model.state.gps.fix = m.fixType == 3 && m.flags.gnssFixOk;
  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;

  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.hAcc; // mm
  _model.state.gps.accuracy.vertical = m.vAcc;   // mm
  _model.state.gps.accuracy.speed = m.sAcc;      // mm/s
  _model.state.gps.accuracy.heading = m.headAcc; // deg * 1e5

  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.hSML;

  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed;
  _model.state.gps.velocity.raw.heading = m.headMot;

  _model.state.gps.velocity.raw.north = m.velN;
  _model.state.gps.velocity.raw.east  = m.velE;
  _model.state.gps.velocity.raw.down  = m.velD;
  _model.state.gps.velocity.raw.speed3d = lrintf(std::sqrt(
    _model.state.gps.velocity.raw.groundSpeed * _model.state.gps.velocity.raw.groundSpeed +
    _model.state.gps.velocity.raw.down * _model.state.gps.velocity.raw.down
  ));

  if(m.valid.validDate && m.valid.validTime)
  {
    _model.state.gps.dateTime.year = m.year;
    _model.state.gps.dateTime.month = m.month;
    _model.state.gps.dateTime.day = m.day;
    _model.state.gps.dateTime.hour = m.hour;
    _model.state.gps.dateTime.minute = m.min;
    _model.state.gps.dateTime.second = m.sec;
    int32_t msec = m.nano / 1000000;
    if(msec < 0) {
      msec += 1000;
    }
    _model.state.gps.dateTime.msec = msec;
  }

  uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;

  calculateHomeVector();
}

void GpsSensor::handleNavSat() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavSat>();
  _model.state.gps.numCh = m.numSvs;
  for (uint8_t i = 0; i < SAT_MAX; i++)
  {
    if(i < m.numSvs)
    {
      _model.state.gps.svinfo[i].id = m.sats[i].svId;
      _model.state.gps.svinfo[i].gnssId = m.sats[i].gnssId;
      _model.state.gps.svinfo[i].cno = m.sats[i].cno;
      _model.state.gps.svinfo[i].quality.value = m.sats[i].flags.value;
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
  else if (std::strcmp(payload + 30, "000A0000") == 0)
  {
    _model.state.gps.support.version = GPS_M10;
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
  if (std::strstr(payload, "GPS") != nullptr)
  {
    _model.state.gps.support.gps = true;
  }
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
  if (std::strstr(payload, "QZSS") != nullptr)
  {
    _model.state.gps.support.qzss = true;
  }
  if (std::strstr(payload, "IMES") != nullptr)
  {
    _model.state.gps.support.imes = true;
  }
  const char* pv = std::strstr(payload, "PROTVER=");
  if (pv != nullptr)
  {
    _model.state.gps.support.protVerMajor = (uint8_t)std::atoi(pv + 8);
  }
}

}
