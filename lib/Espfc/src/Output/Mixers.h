#ifndef _ESPFC_OUTPUT_MIXERS_H_
#define _ESPFC_OUTPUT_MIXERS_H_

namespace Espfc {

enum MixerType {
  MIXER_TRI = 1,
  MIXER_QUADP = 2,
  MIXER_QUADX = 3,
  MIXER_BICOPTER = 4,
  MIXER_GIMBAL = 5,
  MIXER_Y6 = 6,
  MIXER_HEX6 = 7,
  MIXER_FLYING_WING = 8,
  MIXER_Y4 = 9,
  MIXER_HEX6X = 10,
  MIXER_OCTOX8 = 11,
  MIXER_OCTOFLATP = 12,
  MIXER_OCTOFLATX = 13,
  MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
  MIXER_HELI_120_CCPM = 15,
  MIXER_HELI_90_DEG = 16,
  MIXER_VTAIL4 = 17,
  MIXER_HEX6H = 18,
  MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
  MIXER_DUALCOPTER = 20,
  MIXER_SINGLECOPTER = 21,
  MIXER_ATAIL4 = 22,
  MIXER_CUSTOM = 23,
  MIXER_CUSTOM_AIRPLANE = 24,
  MIXER_CUSTOM_TRI = 25,
  MIXER_QUADX_1234 = 26,
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
        case MIXER_QUADX:
          return MixerConfig(4, mixesQuadX);

        case MIXER_QUADX_1234:
          return MixerConfig(4, mixesQuadX1234);

        case MIXER_TRI:
          return MixerConfig(4, mixesTricopter);

        case MIXER_GIMBAL:
          return MixerConfig(2, mixesGimbal);

        case MIXER_CUSTOM:
        case MIXER_CUSTOM_TRI:
        case MIXER_CUSTOM_AIRPLANE:
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
