#pragma once

#include "Device/BusAwareDevice.hpp"
#include "Device/BusDevice.hpp"

namespace Espfc {

enum WirelessDeviceType : uint8_t
{
  WIRELESS_NONE = 0,
  WIRELESS_AUTO = 1,
  WIRELESS_NRF24L01 = 2,
  WIRELESS_MAX
};

namespace Device {

class WirelessDevice : public BusAwareDevice
{
public:
  typedef WirelessDeviceType DeviceType;

  virtual int begin(BusDevice* bus, uint8_t addr) = 0;
  virtual DeviceType getType() const = 0;
  virtual bool testConnection() = 0;

  // Wireless operations
  virtual int send(const uint8_t* data, uint8_t length) = 0;
  virtual int receive(uint8_t* data, uint8_t* length) = 0;
  virtual bool isDataAvailable() = 0;
  virtual int setAddress(const uint8_t* addr, uint8_t length) = 0;
  virtual int setChannel(uint8_t channel) = 0;
  virtual int setPower(uint8_t power) = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Device

} // namespace Espfc
