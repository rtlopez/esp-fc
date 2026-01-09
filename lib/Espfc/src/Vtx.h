#pragma once

namespace Espfc::Connect {

enum VtxDeviceType {
  VTXDEV_UNSUPPORTED = 0, // reserved for MSP
  VTXDEV_RTC6705     = 1,
  // 2 reserved
  VTXDEV_SMARTAUDIO  = 3,
  VTXDEV_TRAMP       = 4,
  VTXDEV_MSP         = 5,
  VTXDEV_UNKNOWN     = 0xFF,
};

enum State {
  INACTIVE,
  INIT,
  SET_POWER,
  SET_CHANNEL,
  IDLE,
};

}