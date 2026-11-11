#include "Output/Mixers.hpp"

namespace Espfc::Output {

MixerConfig Mixers::getMixer(MixerType mixer, const MixerConfig& custom)
{
  // clang-format off
  // default empty mixer
  static MixerEntry mixesEmpty[] = {
    {} // terminator
  };

  // quadX mixer
  static MixerEntry mixesQuadX[] = {
    // RR                              FR                              RL                              FL
    {MIXER_SOURCE_ROLL,   0, -100}, {MIXER_SOURCE_ROLL,   1, -100}, {MIXER_SOURCE_ROLL,   2,  100}, {MIXER_SOURCE_ROLL,   3,  100},
    {MIXER_SOURCE_PITCH,  0,  100}, {MIXER_SOURCE_PITCH,  1, -100}, {MIXER_SOURCE_PITCH,  2,  100}, {MIXER_SOURCE_PITCH,  3, -100},
    {MIXER_SOURCE_YAW,    0, -100}, {MIXER_SOURCE_YAW,    1,  100}, {MIXER_SOURCE_YAW,    2,  100}, {MIXER_SOURCE_YAW,    3, -100},
    {MIXER_SOURCE_THRUST, 0,  100}, {MIXER_SOURCE_THRUST, 1,  100}, {MIXER_SOURCE_THRUST, 2,  100}, {MIXER_SOURCE_THRUST, 3,  100},
    {} // terminator
  };

  // quadX mixer
  static MixerEntry mixesQuadX1234[] = {
    // FL                              FR                              RR                              RL
    {MIXER_SOURCE_ROLL,   0,  100}, {MIXER_SOURCE_ROLL,   1, -100}, {MIXER_SOURCE_ROLL,   2, -100}, {MIXER_SOURCE_ROLL,   3,  100},
    {MIXER_SOURCE_PITCH,  0, -100}, {MIXER_SOURCE_PITCH,  1, -100}, {MIXER_SOURCE_PITCH,  2,  100}, {MIXER_SOURCE_PITCH,  3,  100},
    {MIXER_SOURCE_YAW,    0, -100}, {MIXER_SOURCE_YAW,    1,  100}, {MIXER_SOURCE_YAW,    2, -100}, {MIXER_SOURCE_YAW,    3,  100},
    {MIXER_SOURCE_THRUST, 0,  100}, {MIXER_SOURCE_THRUST, 1,  100}, {MIXER_SOURCE_THRUST, 2,  100}, {MIXER_SOURCE_THRUST, 3,  100},
    {} // terminator
  };

  // tricopter mixer
  static MixerEntry mixesTricopter[] = {
    // FL                              FR                              RMotor                          RServo
    {MIXER_SOURCE_ROLL,   0,    0}, {MIXER_SOURCE_ROLL,   1, -100}, {MIXER_SOURCE_ROLL,   2,  100}, {MIXER_SOURCE_ROLL,   3,    0},
    {MIXER_SOURCE_PITCH,  0,  133}, {MIXER_SOURCE_PITCH,  1,  -67}, {MIXER_SOURCE_PITCH,  2,  -67}, {MIXER_SOURCE_PITCH,  3,    0},
    {MIXER_SOURCE_YAW,    0,    0}, {MIXER_SOURCE_YAW,    1,    0}, {MIXER_SOURCE_YAW,    2,    0}, {MIXER_SOURCE_YAW,    3,  100},
    {MIXER_SOURCE_THRUST, 0,  100}, {MIXER_SOURCE_THRUST, 1,  100}, {MIXER_SOURCE_THRUST, 2,  100}, {MIXER_SOURCE_THRUST, 3,    0},
    {} // terminator
  };

  // tricopter mixer
  static MixerEntry mixesGimbal[] = {
    // L                               R
    {MIXER_SOURCE_PITCH,  0,  100}, {MIXER_SOURCE_PITCH,  1,   100},
    {MIXER_SOURCE_YAW,    0,  100}, {MIXER_SOURCE_YAW,    1,  -100},
    {} // terminator
  };
  // clang-format on

  switch (mixer)
  {
    case FC_MIXER_QUADX:
      return {4, mixesQuadX};

    case FC_MIXER_QUADX_1234:
      return {4, mixesQuadX1234};

    case FC_MIXER_TRI:
      return {4, mixesTricopter};

    case FC_MIXER_GIMBAL:
      return {2, mixesGimbal};

    case FC_MIXER_CUSTOM:
    case FC_MIXER_CUSTOM_TRI:
    case FC_MIXER_CUSTOM_AIRPLANE:
      return custom;

    default:
      return {0, mixesEmpty};
  }
  return {0, mixesEmpty};
}

} // namespace Espfc::Output
