#if defined(ESP8266)

#include "Hal/Wifi.hpp"
#include "Hal/Serial.hpp"
#include <ESP8266WiFi.h>
#include <cstring>

namespace {

static Espfc::Hal::WifiClient client;
static Espfc::Hal::WifiListener* listener = nullptr;

static WiFiServer server(0);
static WiFiClient conn;
static WiFiEventHandler events[4];

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

  // https://github.com/esp8266/Arduino/issues/2545#issuecomment-249222211
  events[0] = WiFi.onStationModeConnected([](const WiFiEventStationModeConnected& ev) {
    if (listener) listener->onStaConnected(ev.ssid.c_str(), ev.channel);
  });
  events[1] = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& ev) {
    if (listener) listener->onStaGotIp(ev.ip.toString().c_str());
  });
  events[2] = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& ev) {
    if (listener) listener->onStaDisconnected();
  });
  events[3] = WiFi.onSoftAPModeStationConnected([](const WiFiEventSoftAPModeStationConnected& ev) {
    if (listener) listener->onApStaConnected(ev.mac);
  });
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
  return (static_cast<uint8_t>(WiFi.getMode()) & static_cast<uint8_t>(WIFI_AP)) != 0;
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
