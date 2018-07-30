#ifndef _ESPFC_WIRELESS_H_
#define _ESPFC_WIRELESS_H_

#include "Model.h"
#include "SerialDeviceAdapter.h"

namespace Espfc {

class Wireless
{
  public:
    Wireless(Model& model): _model(model), _server(1111), _adapter(_client), _initialized(false) {}

    int begin()
    {
      return 1;
    }

    int update()
    {
#if defined(ESP8266)
      return 0;
#endif

      if(_model.config.wireless.mode == WIRELESS_MODE_NULL) return 0;
      //if(!(_model.config.serial[SERIAL_WIFI_0].functionMask & SERIAL_FUNCTION_MSP)) return 0;

      Stats::Measure measure(_model.state.stats, COUNTER_WIFI);

      if(!_initialized && WiFi.status() == WL_CONNECTED)
      {
        _server.begin(_model.config.wireless.port);
        _model.state.serial[SERIAL_WIFI_0].stream = &_adapter;
        _model.state.localIp = WiFi.localIP();
        _initialized = true;
        _model.logger.info().log(F("WIFI CON")).logln(WiFi.localIP());
      }

      if(_initialized && WiFi.status() != WL_CONNECTED)
      {
        _model.state.serial[SERIAL_WIFI_0].stream = nullptr;
        _model.state.localIp = IPAddress(0,0,0,0);
        _initialized = false;
        _model.logger.info().log(F("WIFI DIS"));
      }

      if(!_initialized) return 0;

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
    SerialDeviceAdapter<WiFiClient> _adapter;

    bool _initialized;
};

}

#endif