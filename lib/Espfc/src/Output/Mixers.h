#ifndef _ESPFC_OUTPUT_MIXERS_H_
#define _ESPFC_OUTPUT_MIXERS_H_

namespace Espfc {

enum MixerType {
  FC_MIXER_TRI = 1,
  FC_MIXER_QUADP = 2,
  FC_MIXER_QUADX = 3,
  FC_MIXER_BICOPTER = 4,
  FC_MIXER_GIMBAL = 5,
  FC_MIXER_Y6 = 6,
  FC_MIXER_HEX6 = 7,
  FC_MIXER_FLYING_WING = 8,
  FC_MIXER_Y4 = 9,
  FC_MIXER_HEX6X = 10,
  FC_MIXER_OCTOX8 = 11,
  FC_MIXER_OCTOFLATP = 12,
  FC_MIXER_OCTOFLATX = 13,
  FC_MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
  FC_MIXER_HELI_120_CCPM = 15,
  FC_MIXER_HELI_90_DEG = 16,
  FC_MIXER_VTAIL4 = 17,
  FC_MIXER_HEX6H = 18,
  FC_MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
  FC_MIXER_DUALCOPTER = 20,
  FC_MIXER_SINGLECOPTER = 21,
  FC_MIXER_ATAIL4 = 22,
  FC_MIXER_CUSTOM = 23,
  FC_MIXER_CUSTOM_AIRPLANE = 24,
  FC_MIXER_CUSTOM_TRI = 25,
  FC_MIXER_QUADX_1234 = 26,
};

enum MixerSource {
  MIXER_SOURCE_NULL,
  MIXER_SOURCE_ROLL,
  MIXER_SOURCE_PITCH,
  MIXER_SOURCE_YAW,
  MIXER_SOURCE_THRUST,
  MIXER_SOURCE_RC_ROLL,
  MIXER_SOURCE_RC_PITCH,
  MIXER_SOURCE_RC_YAW,
  MIXER_SOURCE_RC_THRUST,
  MIXER_SOURCE_RC_AUX1,
  MIXER_SOURCE_RC_AUX2,
  MIXER_SOURCE_RC_AUX3,
  MIXER_SOURCE_MAX,
};

enum ThrottleLimitType {
  THROTTLE_LIMIT_TYPE_NONE,
  THROTTLE_LIMIT_TYPE_SCALE,
  THROTTLE_LIMIT_TYPE_CLIP,
  THROTTLE_LIMIT_TYPE_MAX,
};

static const size_t MIXER_RULE_MAX = 64;

class MixerEntry {
  public:
    MixerEntry(): src(MIXER_SOURCE_NULL), dst(0), rate(0) {}
    MixerEntry(int8_t s, int8_t d, int16_t r): src(s), dst(d), rate(r) {}
    int8_t src;
    int8_t dst;
    int16_t rate;
};

class MixerConfig {
  public:
    MixerConfig(): count(0), mixes(NULL) {}
    MixerConfig(const MixerConfig& c): count(c.count), mixes(c.mixes) {}
    MixerConfig(uint8_t c, MixerEntry * m): count(c), mixes(m) {}

    uint8_t count;
    MixerEntry * mixes;
};

namespace Output {

class Mixers
{
  public:
    static MixerConfig getMixer(MixerType mixer, MixerConfig& custom)
    {
      // default empty mixer
      static MixerEntry mixesEmpty[] = {
        MixerEntry() // terminator
      };

      // quadX mixer
      static MixerEntry mixesQuadX[] = {
        // RR                                     FR                                        RL                                        FL
        MixerEntry(MIXER_SOURCE_ROLL,   0, -100), MixerEntry(MIXER_SOURCE_ROLL,   1, -100), MixerEntry(MIXER_SOURCE_ROLL,   2,  100), MixerEntry(MIXER_SOURCE_ROLL,   3,  100),
        MixerEntry(MIXER_SOURCE_PITCH,  0,  100), MixerEntry(MIXER_SOURCE_PITCH,  1, -100), MixerEntry(MIXER_SOURCE_PITCH,  2,  100), MixerEntry(MIXER_SOURCE_PITCH,  3, -100),
        MixerEntry(MIXER_SOURCE_YAW,    0, -100), MixerEntry(MIXER_SOURCE_YAW,    1,  100), MixerEntry(MIXER_SOURCE_YAW,    2,  100), MixerEntry(MIXER_SOURCE_YAW,    3, -100),
        MixerEntry(MIXER_SOURCE_THRUST, 0,  100), MixerEntry(MIXER_SOURCE_THRUST, 1,  100), MixerEntry(MIXER_SOURCE_THRUST, 2,  100), MixerEntry(MIXER_SOURCE_THRUST, 3,  100),
        MixerEntry() // terminator
      };

      // quadX mixer
      static MixerEntry mixesQuadX1234[] = {
        // FL                                     FR                                        RR                                        RL
        MixerEntry(MIXER_SOURCE_ROLL,   0,  100), MixerEntry(MIXER_SOURCE_ROLL,   1, -100), MixerEntry(MIXER_SOURCE_ROLL,   2, -100), MixerEntry(MIXER_SOURCE_ROLL,   3,  100),
        MixerEntry(MIXER_SOURCE_PITCH,  0, -100), MixerEntry(MIXER_SOURCE_PITCH,  1, -100), MixerEntry(MIXER_SOURCE_PITCH,  2,  100), MixerEntry(MIXER_SOURCE_PITCH,  3,  100),
        MixerEntry(MIXER_SOURCE_YAW,    0, -100), MixerEntry(MIXER_SOURCE_YAW,    1,  100), MixerEntry(MIXER_SOURCE_YAW,    2, -100), MixerEntry(MIXER_SOURCE_YAW,    3,  100),
        MixerEntry(MIXER_SOURCE_THRUST, 0,  100), MixerEntry(MIXER_SOURCE_THRUST, 1,  100), MixerEntry(MIXER_SOURCE_THRUST, 2,  100), MixerEntry(MIXER_SOURCE_THRUST, 3,  100),
        MixerEntry() // terminator
      };

      // tricopter mixer
      static MixerEntry mixesTricopter[] = {
        // FL                                     FR                                        RMotor                                    RServo
        MixerEntry(MIXER_SOURCE_ROLL,   0,    0), MixerEntry(MIXER_SOURCE_ROLL,   1, -100), MixerEntry(MIXER_SOURCE_ROLL,   2,  100), MixerEntry(MIXER_SOURCE_ROLL,   3,    0),
        MixerEntry(MIXER_SOURCE_PITCH,  0,  133), MixerEntry(MIXER_SOURCE_PITCH,  1,  -67), MixerEntry(MIXER_SOURCE_PITCH,  2,  -67), MixerEntry(MIXER_SOURCE_PITCH,  3,    0),
        MixerEntry(MIXER_SOURCE_YAW,    0,    0), MixerEntry(MIXER_SOURCE_YAW,    1,    0), MixerEntry(MIXER_SOURCE_YAW,    2,    0), MixerEntry(MIXER_SOURCE_YAW,    3,  100),
        MixerEntry(MIXER_SOURCE_THRUST, 0,  100), MixerEntry(MIXER_SOURCE_THRUST, 1,  100), MixerEntry(MIXER_SOURCE_THRUST, 2,  100), MixerEntry(MIXER_SOURCE_THRUST, 3,    0),
        MixerEntry() // terminator
      };

      // tricopter mixer
      static MixerEntry mixesGimbal[] = {
        // L                                      R
        MixerEntry(MIXER_SOURCE_PITCH,  0,  100), MixerEntry(MIXER_SOURCE_PITCH,  1,   100),
        MixerEntry(MIXER_SOURCE_YAW,    0,  100), MixerEntry(MIXER_SOURCE_YAW,    1,  -100),
        MixerEntry() // terminator
      };

      switch(mixer)
      {
        case FC_MIXER_QUADX:
          return MixerConfig(4, mixesQuadX);

        case FC_MIXER_QUADX_1234:
          return MixerConfig(4, mixesQuadX1234);

        case FC_MIXER_TRI:
          return MixerConfig(4, mixesTricopter);

        case FC_MIXER_GIMBAL:
          return MixerConfig(2, mixesGimbal);

        case FC_MIXER_CUSTOM:
        case FC_MIXER_CUSTOM_TRI:
        case FC_MIXER_CUSTOM_AIRPLANE:
          return custom;

        default:
          return MixerConfig(0, mixesEmpty);
      }
      return MixerConfig(0, mixesEmpty);
    }
};

}

}

#endif
