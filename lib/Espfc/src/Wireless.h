#pragma once

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
    STARTED,
  };
  public:
    Wireless(Model& model);

    int begin();
    int update();

    void startAp();
    int connect();
    void wifiEventConnected(const String& ssid, int channel);
    void wifiEventApConnected(const uint8_t* mac);
    void wifiEventGotIp(const IPAddress& ip);
    void wifiEventDisconnected();

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
