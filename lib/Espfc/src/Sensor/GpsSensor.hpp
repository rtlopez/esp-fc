#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Utils/Timer.h"
#include <GpsParser.hpp>
#include <array>
#include <algorithm>
#include <cstring>

namespace Espfc::Sensor {

class GpsSensor
{
public:
  GpsSensor(Model& model);

  int begin(Device::SerialDevice* port, int baud);

  int update();

private:
  enum State {
    DETECT_BAUD,
    SET_BAUD,
    DISABLE_NMEA,
    GET_VERSION,
    ENABLE_UBX,
    ENABLE_NAV5,
    ENABLE_SBAS,
    SET_RATE,
    WAIT,
    RECEIVE,
    ERROR,
  };

  void handle();

  bool processUbx(uint8_t c);
  void processNmea(uint8_t c);
  void setBaud(int baud);

  void onMessage();

  template<typename MsgType>
  void send(const MsgType m, State ackState, State timeoutState = ERROR)
  {
    Gps::UbxFrame<MsgType> frame{m};
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&frame);
    _port->write(ptr, sizeof(frame));
    
    setState(WAIT, ackState, timeoutState);
  }

  void setState(State state, State ackState, State timeoutState);

  void setState(State state);

  void handleError() const;
  void handleNavPvt() const;
  void handleNavSat() const;
  void handleVersion() const;
  void checkSupport(const char* payload) const;

  static constexpr uint32_t TIMEOUT = 300000;
  static constexpr uint32_t DETECT_TIMEOUT = 2200000;

  Model& _model;

  State _state = WAIT;
  State _ackState = WAIT;
  State _timeoutState = DETECT_BAUD;
  size_t _counter = 0;
  uint32_t _timeout = 0;
  int _currentBaud = 0;
  int _targetBaud = 0;

  Gps::UbxParser _ubxParser;
  Gps::UbxMessage _ubxMsg;

  Gps::NmeaParser _nmeaParser;
  Gps::NmeaMessage _nmeaMsg;

  Device::SerialDevice* _port;
  Utils::Timer _timer;
};

}
