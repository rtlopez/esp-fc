#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"

namespace Espfc::Device
{

class InputIBUS : public InputDevice
{
public:
  struct IBusData
  {
    uint8_t len;
    uint8_t cmd;
    uint16_t ch[14];
    uint16_t checksum;
  } __attribute__((__packed__));

  InputIBUS();

  int begin(Device::SerialDevice *serial);
  InputStatus update() override;
  uint16_t get(uint8_t i) const override;
  void get(uint16_t *data, size_t len) const override;
  size_t getChannelCount() const override;
  bool needAverage() const override;

  void parse(IBusData& frame, int d);

private:
  enum IbusState
  {
    IBUS_LENGTH,
    IBUS_CMD,
    IBUS_DATA,
    IBUS_CRC_LO,
    IBUS_CRC_HI,
  };

  void apply(IBusData& frame);

  static constexpr size_t IBUS_FRAME_SIZE = sizeof(IBusData);
  static constexpr uint8_t IBUS_COMMAND = 0x40;
  static constexpr size_t CHANNELS = 14;

  Device::SerialDevice *_serial;
  IbusState _state;
  uint8_t _idx = 0;
  bool _new_data;

  IBusData _data;
  uint16_t _channels[CHANNELS];
};

}
