#ifndef _ESPFC_WIRELESS_H_
#define _ESPFC_WIRELESS_H_

#include "Model.h"
#include "Cli.h"

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif

namespace Espfc {

class Wireless
{
  public:
    Wireless(Model& model, Cli& cli): _model(model), _cli(cli), _server(1111), _initialized(false) {}

    int begin()
    {
#if defined(ESP8266)
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      _model.logger.info().logln(F("WIFI OFF"));
#elif defined(ESP32)
      WiFi.mode((WiFiMode_t)_model.config.wireless.mode);
      if(_model.config.wireless.mode != WIRELESS_MODE_NULL)
      {
        WiFi.begin(_model.config.wireless.ssid, _model.config.wireless.pass);
      }
      _model.logger.info().log(F("WIFI")).log(FPSTR(WirelessConfig::getModeName((WirelessMode)_model.config.wireless.mode))).log(_model.config.wireless.ssid).logln(_model.config.wireless.pass);
#endif
      return 1;
    }

    int update()
    {
      if(!_initialized && WiFi.status() == WL_CONNECTED)
      {
        _server.begin(_model.config.wireless.port);
        _initialized = true;
      }
      
      if(!_initialized) return 0;

      _model.state.localIp = WiFi.localIP();

      if(!_client.connected())
      {
        _client = _server.available();
        return 0;
      }

      _cli.update(&_client);

      return 1;
    }

  private:
    Model& _model;
    Cli& _cli;
    WiFiServer _server;
    WiFiClient _client;
    bool _initialized;
};

}

#endif