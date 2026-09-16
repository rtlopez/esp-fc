#pragma once

#include "Device/SerialDeviceAdapter.h"
#include "Hal/Wifi.hpp"
#include "Model.h"

#ifdef ESPFC_SERIAL_SOFT_0_WIFI

namespace Espfc {

class Wireless : public Hal::WifiListener
{
  enum Status
  {
    STOPPED,
    STARTED,
  };

public:
  Wireless(Model& model);

  int begin();
  int update();

  void startAp();
  int connect();

  void onStaConnected(const char* ssid, int channel) override;
  void onStaGotIp(const char* ip) override;
  void onStaDisconnected() override;
  void onApStaConnected(const uint8_t* mac) override;

private:
  Model& _model;
  Status _status;
  Device::SerialDeviceAdapter<Hal::WifiClient> _adapter;
};

} // namespace Espfc

#endif
