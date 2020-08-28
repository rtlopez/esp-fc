#ifndef _ESPFC_WIRELESS_H_
#define _ESPFC_WIRELESS_H_

#include "Model.h"
#include "Device/SerialDeviceAdapter.h"
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

namespace Espfc {

class Wireless
{
  public:
    Wireless(Model& model): _model(model), _server(1111), _adapter(_client), _initialized(false) {}

    int begin()
    {
#if defined(ESP32)
      int status = -1;
      if(_model.config.wireless.mode != WIRELESS_MODE_NULL)
      {
        using std::placeholders::_1;
        using std::placeholders::_2;
        WiFi.onEvent(std::function<void(WiFiEvent_t, WiFiEventInfo_t)>(std::bind(&Wireless::wifiEvent, this, _1, _2)));
        WiFi.persistent(false);
        WiFi.mode((WiFiMode_t)_model.config.wireless.mode);
        status = WiFi.begin(_model.config.wireless.ssid, _model.config.wireless.pass);
      }
      else
      {
        WiFi.disconnect();
      }
      const char * modeName = WirelessConfig::getModeName((WirelessMode)_model.config.wireless.mode);
      _model.logger.info().log(F("WIFI")).log(FPSTR(modeName)).log(_model.config.wireless.ssid).log(_model.config.wireless.pass).logln(status);
#elif defined(ESP8266)
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      _model.logger.info().logln(F("WIFI OFF"));
#endif

      return 1;
    }

#if defined(ESP32)
    void wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
    {
      static const char * names[] = {
        "SYSTEM_EVENT_WIFI_READY",               /**< ESP32 WiFi ready */
        "SYSTEM_EVENT_SCAN_DONE",                /**< ESP32 finish scanning AP */
        "SYSTEM_EVENT_STA_START",                /**< ESP32 station start */
        "SYSTEM_EVENT_STA_STOP",                 /**< ESP32 station stop */
        "SYSTEM_EVENT_STA_CONNECTED",            /**< ESP32 station connected to AP */
        "SYSTEM_EVENT_STA_DISCONNECTED",         /**< ESP32 station disconnected from AP */
        "SYSTEM_EVENT_STA_AUTHMODE_CHANGE",      /**< the auth mode of AP connected by ESP32 station changed */
        "SYSTEM_EVENT_STA_GOT_IP",               /**< ESP32 station got IP from connected AP */
        "SYSTEM_EVENT_STA_LOST_IP",              /**< ESP32 station lost IP and the IP is reset to 0 */
        "SYSTEM_EVENT_STA_WPS_ER_SUCCESS",       /**< ESP32 station wps succeeds in enrollee mode */
        "SYSTEM_EVENT_STA_WPS_ER_FAILED",        /**< ESP32 station wps fails in enrollee mode */
        "SYSTEM_EVENT_STA_WPS_ER_TIMEOUT",       /**< ESP32 station wps timeout in enrollee mode */
        "SYSTEM_EVENT_STA_WPS_ER_PIN",           /**< ESP32 station wps pin code in enrollee mode */
        "SYSTEM_EVENT_AP_START",                 /**< ESP32 soft-AP start */
        "SYSTEM_EVENT_AP_STOP",                  /**< ESP32 soft-AP stop */
        "SYSTEM_EVENT_AP_STACONNECTED",          /**< a station connected to ESP32 soft-AP */
        "SYSTEM_EVENT_AP_STADISCONNECTED",       /**< a station disconnected from ESP32 soft-AP */
        "SYSTEM_EVENT_AP_STAIPASSIGNED",         /**< ESP32 soft-AP assign an IP to a connected station */
        "SYSTEM_EVENT_AP_PROBEREQRECVED",        /**< Receive probe request packet in soft-AP interface */
        "SYSTEM_EVENT_GOT_IP6",                  /**< ESP32 station or ap or ethernet interface v6IP addr is preferred */
        "SYSTEM_EVENT_ETH_START",                /**< ESP32 ethernet start */
        "SYSTEM_EVENT_ETH_STOP",                 /**< ESP32 ethernet stop */
        "SYSTEM_EVENT_ETH_CONNECTED",            /**< ESP32 ethernet phy link up */
        "SYSTEM_EVENT_ETH_DISCONNECTED",         /**< ESP32 ethernet phy link down */
        "SYSTEM_EVENT_ETH_GOT_IP"                /**< ESP32 ethernet got IP from connected AP */
      };
      
      switch(event)
      {
        case SYSTEM_EVENT_STA_GOT_IP:
          _server.begin(_model.config.wireless.port);
          _model.state.serial[SERIAL_SOFT_0].stream = &_adapter;
          _model.state.localIp = WiFi.localIP();
          _initialized = true;
          _model.logger.info().log(F("WIFI EV")).log(F("IP")).logln(WiFi.localIP());
          break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
          _model.state.serial[SERIAL_SOFT_0].stream = nullptr;
          _model.state.localIp = IPAddress(0,0,0,0);
          _initialized = false;
          _model.logger.info().log(F("WIFI EV")).logln(F("DISCONNECTED"));
          break;
        default:
          _model.logger.info().log(F("WIFI EV")).logln(names[event]);
          break;
      }
    }
#endif

    int update()
    {
#if defined(ESP8266)
      return 0;
#endif

      if(_model.config.wireless.mode == WIRELESS_MODE_NULL) return 0;
      if(!_initialized) return 0;
      Stats::Measure measure(_model.state.stats, COUNTER_WIFI);

      if(_server.hasClient())
      {
        _client = _server.available();
      }

      return 1;
    }

  private:
    Model& _model;
    WiFiServer _server;
    WiFiClient _client;
    Device::SerialDeviceAdapter<WiFiClient> _adapter;

    bool _initialized;
};

}

#endif