#pragma once

#include "Hal/Serial.hpp"
#include <cstddef>
#include <cstdint>

namespace Espfc::Hal {

enum class WifiMode : uint8_t
{
  Off = 0,
  Sta = 1,
  Ap = 2,
  ApSta = 3,
};

struct WifiStatus
{
  char staIp[16] = {0};
  char apIp[16] = {0};
  char staMac[18] = {0};
  char apMac[18] = {0};
  WifiMode mode = WifiMode::Off;
  int state = 0;
  int channel = 0;
};

// Framework independent sink for WiFi events, no Arduino types allowed here.
class WifiListener
{
public:
  virtual void onStaConnected(const char* ssid, int channel) = 0;
  virtual void onStaGotIp(const char* ip) = 0;
  virtual void onStaDisconnected() = 0;
  virtual void onApStaConnected(const uint8_t* mac) = 0;

protected:
  ~WifiListener() = default;
};

// Accepted TCP connection, method set compatible with Device::SerialDeviceAdapter.
class WifiClient
{
public:
  void begin(const SerialDeviceConfig& conf);
  void updateBaudRate(int baud);
  int available();
  int read();
  size_t readMany(uint8_t* c, size_t l);
  int peek();
  void flush();
  size_t write(uint8_t c);
  size_t write(const uint8_t* c, size_t l);
  int availableForWrite();
  bool isTxFifoEmpty();
};

class Wifi
{
public:
  static bool begin();
  static void setListener(WifiListener* listener);
  static bool startAp(const char* ssid);
  static bool startSta(const char* ssid, const char* pass);
  static bool isApActive();
  static void getStatus(WifiStatus& status);
  static void stop();

  static bool listen(uint16_t port);
  static bool accept();
};

WifiClient* getWifiClient();

} // namespace Espfc::Hal
