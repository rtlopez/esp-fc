#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include <cstdint>
#include <cstddef>

namespace Espfc {

namespace Device {

struct SbusData
{
  uint8_t syncByte;
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
  uint8_t flags;
  /**
   * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
   * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
   * and https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
   */
  uint8_t endByte;
} __attribute__ ((__packed__));

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

class InputSBUS: public InputDevice
{
  public:
    enum SbusState {
      SBUS_START,
      SBUS_DATA,
      SBUS_END
    };

    InputSBUS();

    int begin(Device::SerialDevice * serial);
    InputStatus update() override;
    uint16_t get(uint8_t i) const override;
    void get(uint16_t * data, size_t len) const override;
    size_t getChannelCount() const override;
    bool needAverage() const override;

  private:
    void parse(int d);
    void apply();
    uint16_t convert(int v);

    static constexpr size_t SBUS_FRAME_SIZE = sizeof(SbusData);
    static constexpr size_t CHANNELS = 16;

    Device::SerialDevice * _serial;
    SbusState _state;
    uint8_t _idx = 0;
    bool _new_data;

    uint8_t _data[SBUS_FRAME_SIZE];
    uint16_t _channels[CHANNELS];
    uint8_t _flags;
};

}

}
