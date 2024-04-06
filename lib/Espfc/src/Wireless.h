#ifndef _ESPFC_WIRELESS_H_
#define _ESPFC_WIRELESS_H_

#include "Model.h"
#include "Device/SerialDeviceAdapter.h"

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

namespace Espfc {

class Wireless
{
  enum Status {
    STOPPED,
    STARTING,
    STARTED,
  };
  public:
    Wireless(Model& model): _model(model), _status(STOPPED), _server(1111), _adapter(_client) {}

    int begin()
    {
      WiFi.persistent(false);
#ifdef ESPFC_ESPNOW
      if(_model.isActive(FEATURE_RX_SPI))
      {
        startAp();
      }
#endif
      return 1;
    }

    void startAp()
    {
      bool status = WiFi.softAP("ESP-FC");
      _model.logger.info().log(F("WIFI AP")).logln(status);
    }

    int connect()
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
      //delay(200);
      _server.begin(_model.config.wireless.port);
      _server.setNoDelay(true);
      _model.logger.info().log(F("WIFI PORT")).log(WiFi.status()).logln(_model.config.wireless.port);
      return 1;
    }

    void wifiEventConnected(const String& ssid, int channel)
    {
      _model.logger.info().log(F("WIFI CONNECTED")).log(ssid).logln(channel);
    }

    void wifiEventApConnected(const uint8_t* mac)
    {
      char buf[20];
      snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      _model.logger.info().log(F("WIFI AP CONNECTED")).logln(buf);
      //_server.begin(_model.config.wireless.port);
      //_model.logger.info().log(F("WIFI PORT")).log(WiFi.status()).logln(_model.config.wireless.port);
    }

    void wifiEventGotIp(const IPAddress& ip)
    {
      _model.state.localIp = ip;
      //_status = STARTED;
      _model.state.serial[SERIAL_SOFT_0].stream = &_adapter;
      _model.logger.info().log(F("WIFI IP")).logln(_model.state.localIp.toString());
    }

    void wifiEventDisconnected()
    {
      _model.state.serial[SERIAL_SOFT_0].stream = nullptr;
      _model.state.localIp = IPAddress(0,0,0,0);
      //_status = STOPPED;
      _model.logger.info().logln(F("WIFI DISCONNECTED"));
    }

    int update()
    {
      Stats::Measure measure(_model.state.stats, COUNTER_WIFI);

      switch(_status)
      {
        case STOPPED: 
          if(_model.state.rescueConfigMode == RESCUE_CONFIG_ACTIVE)
          {
            _status = STARTING;
            connect();
            return 1;
          }
          break;
        case STARTING:
          /*if(WiFi.status() == WL_CONNECTED)
          {
            _status = STARTED;
            _server.begin(_model.config.wireless.port);
            _model.logger.info().log(F("WIFI PORT")).log(WiFi.status()).logln(_model.config.wireless.port);
          }*/
          break;
        case STARTED:
          break;
      }

      if(_server.hasClient())
      {
        _client = _server.accept();
      }

      return 1;
    }

  private:
    Model& _model;
    Status _status;
    WiFiServer _server;
    WiFiClient _client;
    Device::SerialDeviceAdapter<WiFiClient> _adapter;
#ifdef ESPFC_WIFI_ALT
    WiFiEventHandler _events[4];
#endif
};

}

#endif

#endif