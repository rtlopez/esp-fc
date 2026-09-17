#if defined(ESP32)

#include "Hal/Wifi.hpp"
#include "Hal/Serial.hpp"
#include <WiFi.h>
#include <algorithm>
#include <cstring>

namespace {

static Espfc::Hal::WifiClient client;
static Espfc::Hal::WifiListener* listener = nullptr;

static WiFiServer server(0);
static WiFiClient conn;

static void copyStr(char* dst, size_t len, const char* src)
{
  std::strncpy(dst, src, len - 1);
  dst[len - 1] = 0;
}

static Espfc::Hal::WifiMode toMode(uint8_t mode)
{
  switch (mode & 0x3)
  {
    case 1:
      return Espfc::Hal::WifiMode::Sta;
    case 2:
      return Espfc::Hal::WifiMode::Ap;
    case 3:
      return Espfc::Hal::WifiMode::ApSta;
  }
  return Espfc::Hal::WifiMode::Off;
}

} // namespace

namespace Espfc::Hal {

WifiClient* getWifiClient()
{
  return &client;
}

bool Wifi::begin()
{
  WiFi.persistent(false);
  return true;
}

void Wifi::setListener(WifiListener* l)
{
  listener = l;

  WiFi.onEvent(
      [](WiFiEvent_t ev, WiFiEventInfo_t info) {
        if (!listener) return;
        // ssid is not zero terminated, length is passed separately
        char ssid[33];
        const size_t len = std::min((size_t)info.wifi_sta_connected.ssid_len, sizeof(ssid) - 1);
        std::memcpy(ssid, info.wifi_sta_connected.ssid, len);
        ssid[len] = 0;
        listener->onStaConnected(ssid, info.wifi_sta_connected.channel);
      },
      ARDUINO_EVENT_WIFI_STA_CONNECTED);

  WiFi.onEvent(
      [](WiFiEvent_t ev, WiFiEventInfo_t info) {
        if (!listener) return;
        char ip[16];
        copyStr(ip, sizeof(ip), IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
        listener->onStaGotIp(ip);
      },
      ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFi.onEvent(
      [](WiFiEvent_t ev, WiFiEventInfo_t info) {
        if (listener) listener->onStaDisconnected();
      },
      ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.onEvent(
      [](WiFiEvent_t ev, WiFiEventInfo_t info) {
        if (listener) listener->onApStaConnected(info.wifi_ap_staconnected.mac);
      },
      ARDUINO_EVENT_WIFI_AP_STACONNECTED);
}

bool Wifi::startAp(const char* ssid)
{
  return WiFi.softAP(ssid);
}

bool Wifi::startSta(const char* ssid, const char* pass)
{
  return WiFi.begin(ssid, pass) != WL_CONNECT_FAILED;
}

bool Wifi::isApActive()
{
  return (static_cast<uint8_t>(WiFi.getMode()) & static_cast<uint8_t>(WIFI_MODE_AP)) != 0;
}

void Wifi::getStatus(WifiStatus& status)
{
  copyStr(status.staIp, sizeof(status.staIp), WiFi.localIP().toString().c_str());
  copyStr(status.apIp, sizeof(status.apIp), WiFi.softAPIP().toString().c_str());
  copyStr(status.staMac, sizeof(status.staMac), WiFi.macAddress().c_str());
  copyStr(status.apMac, sizeof(status.apMac), WiFi.softAPmacAddress().c_str());
  status.mode = toMode(static_cast<uint8_t>(WiFi.getMode()));
  status.state = static_cast<int>(WiFi.status());
  status.channel = WiFi.channel();
}

void Wifi::stop()
{
  WiFi.disconnect();
  WiFi.softAPdisconnect();
}

bool Wifi::listen(uint16_t port)
{
  server.begin(port);
  server.setNoDelay(true);
  return true;
}

bool Wifi::accept()
{
  if (!server.hasClient()) return false;
  conn = server.accept();
  return true;
}

void WifiClient::begin(const SerialDeviceConfig& conf)
{
  // noop
}

void WifiClient::updateBaudRate(int baud)
{
  // noop
}

int WifiClient::available()
{
  return conn.available();
}

int WifiClient::read()
{
  return conn.read();
}

size_t WifiClient::readMany(uint8_t* c, size_t l)
{
  const int res = conn.read(c, l);
  return res > 0 ? (size_t)res : 0;
}

int WifiClient::peek()
{
  return conn.peek();
}

void WifiClient::flush()
{
  conn.flush();
}

size_t WifiClient::write(uint8_t c)
{
  return conn.write(c);
}

size_t WifiClient::write(const uint8_t* c, size_t l)
{
  return conn.write(c, l);
}

int WifiClient::availableForWrite()
{
  return SERIAL_TX_BUFFER_SIZE;
}

bool WifiClient::isTxFifoEmpty()
{
  return true;
}

} // namespace Espfc::Hal

#endif
