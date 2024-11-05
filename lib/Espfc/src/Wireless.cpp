#include "Wireless.h"

#ifdef ESPFC_SERIAL_SOFT_0_WIFI

namespace Espfc {

Wireless::Wireless(Model& model): _model(model), _status(STOPPED), _server(1111), _adapter(_client) {}

int Wireless::begin()
{
  WiFi.persistent(false);
#ifdef ESPFC_ESPNOW
  if(_model.isFeatureActive(FEATURE_RX_SPI))
  {
    startAp();
  }
#endif
  return 1;
}

void Wireless::startAp()
{
  bool status = WiFi.softAP("ESP-FC");
  _model.logger.info().log(F("WIFI AP START")).logln(status);
}

int Wireless::connect()
{
#ifdef ESPFC_WIFI_ALT
  // https://github.com/esp8266/Arduino/issues/2545#issuecomment-249222211
  _events[0] = WiFi.onStationModeConnected([this](const WiFiEventStationModeConnected& ev) { this->wifiEventConnected(ev.ssid, ev.channel); });
  _events[1] = WiFi.onStationModeGotIP([this](const WiFiEventStationModeGotIP& ev) { this->wifiEventGotIp(ev.ip); });
  _events[2] = WiFi.onStationModeDisconnected([this](const WiFiEventStationModeDisconnected& ev) { this->wifiEventDisconnected(); });
  _events[3] = WiFi.onSoftAPModeStationConnected([this](const WiFiEventSoftAPModeStationConnected& ev) { this->wifiEventApConnected(ev.mac); });
#elif defined(ESPFC_WIFI)
  WiFi.onEvent([this](WiFiEvent_t ev, WiFiEventInfo_t info) { 
    this->wifiEventConnected(String(info.wifi_sta_connected.ssid, info.wifi_sta_connected.ssid_len), info.wifi_sta_connected.channel);
  }, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent([this](WiFiEvent_t ev, WiFiEventInfo_t info) { this->wifiEventGotIp(IPAddress(info.got_ip.ip_info.ip.addr)); }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent([this](WiFiEvent_t ev, WiFiEventInfo_t info) { this->wifiEventDisconnected(); }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
#endif
  if(_model.config.wireless.ssid[0] != 0)
  {
    WiFi.begin(_model.config.wireless.ssid, _model.config.wireless.pass);
    _model.logger.info().log(F("WIFI STA")).log(_model.config.wireless.ssid).log(_model.config.wireless.pass).log(WiFi.getMode()).logln(WiFi.status());
  }
  if(!(WiFi.getMode() & WIFI_AP))
  {
    startAp();
  }
  _server.begin(_model.config.wireless.port);
  _server.setNoDelay(true);
  _model.state.serial[SERIAL_SOFT_0].stream = &_adapter;
  _model.logger.info().log(F("WIFI SERVER PORT")).log(WiFi.status()).logln(_model.config.wireless.port);
  return 1;
}

void Wireless::wifiEventConnected(const String& ssid, int channel)
{
  _model.logger.info().log(F("WIFI STA CONN")).log(ssid).logln(channel);
}

void Wireless::wifiEventApConnected(const uint8_t* mac)
{
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  _model.logger.info().log(F("WIFI AP CONNECT")).logln(buf);
}

void Wireless::wifiEventGotIp(const IPAddress& ip)
{
  _model.logger.info().log(F("WIFI STA IP")).logln(ip.toString());
}

void Wireless::wifiEventDisconnected()
{
  _model.logger.info().logln(F("WIFI STA DISCONNECT"));
}

int Wireless::update()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_WIFI);

  switch(_status)
  {
    case STOPPED:
      if(_model.state.mode.rescueConfigMode == RESCUE_CONFIG_ACTIVE && _model.isFeatureActive(FEATURE_SOFTSERIAL))
      {
        connect();
        _status = STARTED;
        return 1;
      }
      break;
    case STARTED:
      if(_server.hasClient())
      {
        _client = _server.accept();
      }
      break;
  }

  return 1;
}

}

#endif
