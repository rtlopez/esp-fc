#ifndef _ESPFC_CLI_H_
#define _ESPFC_CLI_H_

#include <cstring>
#include <cctype>

#if defined(ESP8266)
extern "C" {
#include "user_interface.h"
}
#endif //ESP8266

#include "Model.h"
#include "Msp.h"
#include "Hardware.h"
#include "Logger.h"
#include "Device/GyroDevice.h"

#if defined(ESP32)
#include <freertos/task.h>
#include <WiFi.h>
#endif

namespace Espfc {

class Cli
{
  public:
    class Cmd
    {
      public:
        Cmd() { for(size_t i = 0; i < ARGS_SIZE; ++i) args[i] = NULL; }
        static const size_t ARGS_SIZE = 12;
        const char * args[ARGS_SIZE];
    };

    enum ParamType {
      PARAM_NONE,   // unused
      PARAM_BOOL,   // boolean
      PARAM_BYTE,   // 8 bit int
      PARAM_BYTE_U, // 8 bit uint
      PARAM_SHORT,  // 16 bit int
      PARAM_INT,    // 32 bit int
      PARAM_FLOAT,  // 32 bit float
      PARAM_INPUT_CHANNEL,  // input channel config
      PARAM_OUTPUT_CHANNEL,  // output channel config
      PARAM_SCALER,  // scaler config
      PARAM_MODE,    // scaler config
      PARAM_MIXER,   // mixer config
      PARAM_STRING,  // string
    };

    class Param
    {
      public:
        Param(): Param(NULL, PARAM_NONE, NULL, NULL) {}
        Param(const Param& p): Param(p.name, p.type, p.addr, p.choices) {}

        Param(const char * n, ParamType t, char * a, const char ** c): name(n), type(t), addr(a), choices(c) {}

        Param(const char * n, bool    * a): Param(n, PARAM_BOOL,   reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, int8_t  * a): Param(n, PARAM_BYTE,   reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, uint8_t * a): Param(n, PARAM_BYTE_U, reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, int16_t * a): Param(n, PARAM_SHORT,  reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, int32_t * a): Param(n, PARAM_INT,    reinterpret_cast<char*>(a), NULL) {}

        Param(const char * n, int8_t * a, const char ** c): Param(n, PARAM_BYTE, reinterpret_cast<char*>(a), c) {}
        Param(const char * n, InputChannelConfig * a):  Param(n, PARAM_INPUT_CHANNEL,  reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, OutputChannelConfig * a): Param(n, PARAM_OUTPUT_CHANNEL, reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, ScalerConfig * a):        Param(n, PARAM_SCALER, reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, ActuatorCondition * a):   Param(n, PARAM_MODE,   reinterpret_cast<char*>(a), NULL) {}
        Param(const char * n, MixerEntry * a):          Param(n, PARAM_MIXER,  reinterpret_cast<char*>(a), NULL) {}

        void print(Stream& stream) const
        {
          if(!addr)
          {
            stream.print(F("UNSET"));
            return;
          }
          switch(type)
          {
            case PARAM_NONE:
              stream.print(F("NONE"));
              break;
            case PARAM_BOOL:
              stream.print(*addr != 0);
              break;
            case PARAM_BYTE:
              print(stream, *reinterpret_cast<int8_t*>(addr));
              break;
            case PARAM_BYTE_U:
              print(stream, *reinterpret_cast<uint8_t*>(addr));
              break;
            case PARAM_SHORT:
              print(stream, *reinterpret_cast<int16_t*>(addr));
              break;
            case PARAM_INT:
              print(stream, *reinterpret_cast<int32_t*>(addr));
              break;
            case PARAM_FLOAT:
              stream.print(*reinterpret_cast<float*>(addr), 4);
              break;
            case PARAM_STRING:
              stream.print(addr);
              break;
            case PARAM_INPUT_CHANNEL:
              print(stream, *reinterpret_cast<InputChannelConfig*>(addr));
              break;
            case PARAM_OUTPUT_CHANNEL:
              print(stream, *reinterpret_cast<OutputChannelConfig*>(addr));
              break;
            case PARAM_SCALER:
              print(stream, *reinterpret_cast<ScalerConfig*>(addr));
              break;
            case PARAM_MODE:
              print(stream, *reinterpret_cast<ActuatorCondition*>(addr));
              break;
            case PARAM_MIXER:
              print(stream, *reinterpret_cast<MixerEntry*>(addr));
              break;
          }
        }

        void print(Stream& stream, const OutputChannelConfig& och) const
        {
          stream.print(och.servo ? 'S' : 'M');
          stream.print(' ');
          stream.print(och.reverse ? 'R' : 'N');
          stream.print(' ');
          stream.print(och.min);
          stream.print(' ');
          stream.print(och.neutral);
          stream.print(' ');
          stream.print(och.max);
        }

        void print(Stream& stream, const InputChannelConfig& ich) const
        {
          stream.print(ich.map);
          stream.print(' ');
          stream.print(ich.min);
          stream.print(' ');
          stream.print(ich.neutral);
          stream.print(' ');
          stream.print(ich.max);
          stream.print(' ');
          stream.print(ich.fsMode == 0 ? 'A' : (ich.fsMode == 1 ? 'H' : (ich.fsMode == 2 ? 'S' : '?')));
          stream.print(' ');
          stream.print(ich.fsValue);
        }

        void print(Stream& stream, const ScalerConfig& sc) const
        {
          stream.print(sc.dimension);
          stream.print(' ');
          stream.print(sc.channel);
          stream.print(' ');
          stream.print(sc.minScale);
          stream.print(' ');
          stream.print(sc.maxScale);
        }

        void print(Stream& stream, const ActuatorCondition& ac) const
        {
          stream.print(ac.id);
          stream.print(' ');
          stream.print(ac.ch);
          stream.print(' ');
          stream.print(ac.min);
          stream.print(' ');
          stream.print(ac.max);
        }

        void print(Stream& stream, const MixerEntry& me) const
        {
          stream.print(me.src);
          stream.print(' ');
          stream.print(me.dst);
          stream.print(' ');
          stream.print(me.rate);
        }

        void print(Stream& stream, int32_t v) const
        {
          if(choices)
          {
            for(int32_t i = 0; choices[i]; i++)
            {
              if(i == v)
              {
                stream.print(FPSTR(choices[i]));
                return;
              }
            }
          }
          stream.print(v);
        }

        void update(const char ** args) const
        {
          const char * v = args[2];
          if(!addr || !v) return;
          switch(type)
          {
            case PARAM_BOOL:
              if(*v == '0') *addr = 0;
              if(*v == '1') *addr = 1;
              break;
            case PARAM_BYTE:
              write((int8_t)parse(v));
              break;
            case PARAM_BYTE_U:
              write((uint8_t)parse(v));
              break;
            case PARAM_SHORT:
              write((int16_t)parse(v));
              break;
            case PARAM_INT:
              write((int32_t)parse(v));
              break;
            case PARAM_FLOAT:
              write(String(v).toFloat());
              break;
            case PARAM_STRING:
              write(String(v));
              break;
            case PARAM_OUTPUT_CHANNEL:
              write(*reinterpret_cast<OutputChannelConfig*>(addr), args);
              break;
            case PARAM_INPUT_CHANNEL:
              write(*reinterpret_cast<InputChannelConfig*>(addr), args);
              break;
            case PARAM_SCALER:
              write(*reinterpret_cast<ScalerConfig*>(addr), args);
              break;
            case PARAM_MODE:
              write(*reinterpret_cast<ActuatorCondition*>(addr), args);
              break;
            case PARAM_MIXER:
              write(*reinterpret_cast<MixerEntry*>(addr), args);
              break;
            case PARAM_NONE:
              break;
          }
        }

        void write(OutputChannelConfig& och, const char ** args) const
        {
          if(args[2]) och.servo = *args[2] == 'S';
          if(args[3]) och.reverse = *args[3] == 'R';
          if(args[4]) och.min = String(args[4]).toInt();
          if(args[5]) och.neutral = String(args[5]).toInt();
          if(args[6]) och.max = String(args[6]).toInt();
        }

        void write(InputChannelConfig& ich, const char ** args) const
        {
          if(args[2]) ich.map = String(args[2]).toInt();
          if(args[3]) ich.min = String(args[3]).toInt();
          if(args[4]) ich.neutral = String(args[4]).toInt();
          if(args[5]) ich.max = String(args[5]).toInt();
          if(args[6]) ich.fsMode = *args[6] == 'A' ? 0 : (*args[6] == 'H' ? 1 : (*args[6] == 'S' ? 2 : 0));
          if(args[7]) ich.fsValue = String(args[7]).toInt();
        }

        void write(ScalerConfig& sc, const char ** args) const
        {
          if(args[2]) sc.dimension = (ScalerDimension)String(args[2]).toInt();
          if(args[3]) sc.channel = String(args[3]).toInt();
          if(args[4]) sc.minScale = String(args[4]).toInt();
          if(args[5]) sc.maxScale = String(args[5]).toInt();
        }

        void write(ActuatorCondition& ac, const char ** args) const
        {
          if(args[2]) ac.id = String(args[2]).toInt();
          if(args[3]) ac.ch = String(args[3]).toInt();
          if(args[4]) ac.min = String(args[4]).toInt();
          if(args[5]) ac.max = String(args[5]).toInt();
        }

        void write(MixerEntry& ac, const char ** args) const
        {
          if(args[2]) ac.src = Math::bound((int)String(args[2]).toInt(), 0, MIXER_SOURCE_MAX - 1);
          if(args[3]) ac.dst = Math::bound((int)String(args[3]).toInt(), 0, (int)(OUTPUT_CHANNELS - 1));
          if(args[4]) ac.rate = Math::bound((int)String(args[4]).toInt(), -1000, 1000);
        }

        template<typename T>
        void write(const T v) const
        {
          *reinterpret_cast<T*>(addr) = v;
        }

        void write(const String& v) const
        {
          strncat(addr, v.c_str(), 16);
        }

        int32_t parse(const char * v) const
        {
          if(choices)
          {
            for(int32_t i = 0; choices[i]; i++)
            {
              if(strcasecmp_P(v, choices[i]) == 0) return i;
            }
          }
          String tmp = v;
          return tmp.toInt();
        }

        const char * name;
        ParamType type;
        char * addr;
        const char ** choices;
    };

    Cli(Model& model): _model(model), _index(0), _msp(model), _ignore(false), _active(false)
    {
      _params = initialize(_model.config);
    }

    static const Param * initialize(ModelConfig& c)
    {
      const char ** busDevChoices            = Device::BusDevice::getNames();
      const char ** gyroDevChoices           = Device::GyroDevice::getNames();
      static const char* gyroDlpfChoices[]   = { PSTR("256Hz"), PSTR("188Hz"), PSTR("98Hz"), PSTR("42Hz"), PSTR("20Hz"), PSTR("10Hz"), PSTR("5Hz"), PSTR("EXPERIMENTAL"), NULL };
      static const char* accelModeChoices[]  = { PSTR("DELAYED"), PSTR("GYRO"), NULL };
      //static const char* magDevChoices[]    = { PSTR("AUTO"), PSTR("NONE"), PSTR("HMC5883"), NULL };
      //static const char* magRateChoices[]   = { PSTR("3Hz"), PSTR("7P5Hz"), PSTR("15hz"), PSTR("30Hz"), PSTR("75hz"), NULL };
      const char ** fusionModeChoices        = FusionConfig::getModeNames();
      static const char* debugModeChoices[]  = { PSTR("NONE"), PSTR("CYCLETIME"), PSTR("BATTERY"), PSTR("GYRO"),
                                                 PSTR("ACCELEROMETER"), PSTR("MIXER"), PSTR("AIRMODE"), PSTR("PIDLOOP"),
                                                 PSTR("NOTCH"), PSTR("RC_INTERPOLATION"), PSTR("VELOCITY"), PSTR("DTERM_FILTER"),
                                                 PSTR("ANGLERATE"), PSTR("ESC_SENSOR"), PSTR("SCHEDULER"), PSTR("STACK"),
                                                 PSTR("ESC_SENSOR_RPM"), PSTR("ESC_SENSOR_TMP"), PSTR("ALTITUDE"), PSTR("FFT"),
                                                 PSTR("FFT_TIME"), PSTR("FFT_FREQ"), PSTR("FRSKY_D_RX"), NULL };
      static const char* filterTypeChoices[] = { PSTR("PT1"), PSTR("BIQUAD"), PSTR("FIR"), PSTR("NOTCH"), PSTR("FIR2"), PSTR("NONE"), NULL };
      static const char* alignChoices[]      = { PSTR("DEFAULT"), PSTR("CW0"), PSTR("CW90"), PSTR("CW180"), PSTR("CW270"), PSTR("CW0_FLIP"), PSTR("CW90_FLIP"), PSTR("CW180_FLIP"), PSTR("CW270_FLIP"), NULL };
      static const char* mixerTypeChoices[]  = { PSTR("NONE"), PSTR("TRI"), PSTR("QUADP"), PSTR("QUADX"), PSTR("BI"),
                                                 PSTR("GIMBAL"), PSTR("Y6"), PSTR("HEX6"), PSTR("FWING"), PSTR("Y4"),
                                                 PSTR("HEX6X"), PSTR("OCTOX8"), PSTR("OCTOFLATP"), PSTR("OCTOFLATX"), PSTR("AIRPLANE"),
                                                 PSTR("HELI120"), PSTR("HELI90"), PSTR("VTAIL4"), PSTR("HEX6H"), PSTR("PPMSERVO"),
                                                 PSTR("DUALCOPTER"), PSTR("SINGLECOPTER"), PSTR("ATAIL4"), PSTR("CUSTOM"), PSTR("CUSTOMAIRPLANE"),
                                                 PSTR("CUSTOMTRI"), PSTR("QUADX1234"), NULL };
      static const char* interpolChoices[]   = { PSTR("NONE"), PSTR("DEFAULT"), PSTR("AUTO"), PSTR("MANUAL"), NULL };
      static const char* protocolChoices[]   = { PSTR("PWM"), PSTR("ONESHOT125"), PSTR("ONESHOT42"), PSTR("MULTISHOT"), PSTR("BRUSHED"),
                                                 PSTR("DSHOT150"), PSTR("DSHOT300"), PSTR("DSHOT600"), PSTR("DSHOT1200"), PSTR("PROSHOT1000"), NULL };

      const char ** wifiModeChoices            = WirelessConfig::getModeNames();

      size_t i = 0;
      static const Param params[] = {

        Param(PSTR("features"), &c.featureMask),
        Param(PSTR("debug_mode"), &c.debugMode, debugModeChoices),

        Param(PSTR("gyro_bus"), &c.gyroBus, busDevChoices),
        Param(PSTR("gyro_dev"), &c.gyroDev, gyroDevChoices),
        Param(PSTR("gyro_dlpf"), &c.gyroDlpf, gyroDlpfChoices),
        Param(PSTR("gyro_sync"), &c.gyroSync),
        Param(PSTR("gyro_align"), &c.gyroAlign, alignChoices),
        Param(PSTR("gyro_lpf_type"), &c.gyroFilter.type, filterTypeChoices),
        Param(PSTR("gyro_lpf_freq"), &c.gyroFilter.freq),
        Param(PSTR("gyro_lpf2_type"), &c.gyroFilter2.type, filterTypeChoices),
        Param(PSTR("gyro_lpf2_freq"), &c.gyroFilter2.freq),
        Param(PSTR("gyro_lpf3_type"), &c.gyroFilter3.type, filterTypeChoices),
        Param(PSTR("gyro_lpf3_freq"), &c.gyroFilter3.freq),
        Param(PSTR("gyro_notch1_freq"), &c.gyroNotch1Filter.freq),
        Param(PSTR("gyro_notch1_cutoff"), &c.gyroNotch1Filter.cutoff),
        Param(PSTR("gyro_notch2_freq"), &c.gyroNotch2Filter.freq),
        Param(PSTR("gyro_notch2_cutoff"), &c.gyroNotch2Filter.cutoff),
        Param(PSTR("gyro_offset_x"), &c.gyroBias[0]),
        Param(PSTR("gyro_offset_y"), &c.gyroBias[1]),
        Param(PSTR("gyro_offset_z"), &c.gyroBias[2]),

        Param(PSTR("accel_bus"), &c.accelBus, busDevChoices),
        Param(PSTR("accel_dev"), &c.accelDev, gyroDevChoices),
        Param(PSTR("accel_mode"), &c.accelMode, accelModeChoices),
        Param(PSTR("accel_align"), &c.accelAlign, alignChoices),
        Param(PSTR("accel_lpf_type"), &c.accelFilter.type, filterTypeChoices),
        Param(PSTR("accel_lpf_freq"), &c.accelFilter.freq),
        Param(PSTR("accel_offset_x"), &c.accelBias[0]),
        Param(PSTR("accel_offset_y"), &c.accelBias[1]),
        Param(PSTR("accel_offset_z"), &c.accelBias[2]),

        /*
        Param(PSTR("mag_bus"), &c.magBus, busDevChoices),
        Param(PSTR("mag_dev"), &c.magDev, magDevChoices),
        Param(PSTR("mag_rate"), &c.magSampleRate, magRateChoices),
        Param(PSTR("mag_align"), &c.magAlign, alignChoices),
        Param(PSTR("mag_filter_type"), &c.magFilter.type),
        Param(PSTR("mag_filter_lpf"), &c.magFilter.freq),
        Param(PSTR("mag_offset_x"), &c.magCalibrationOffset[0]),
        Param(PSTR("mag_offset_y"), &c.magCalibrationOffset[1]),
        Param(PSTR("mag_offset_z"), &c.magCalibrationOffset[2]),
        Param(PSTR("mag_scale_x"), &c.magCalibrationScale[0]),
        Param(PSTR("mag_scale_y"), &c.magCalibrationScale[1]),
        Param(PSTR("mag_scale_z"), &c.magCalibrationScale[2]),
        */

        Param(PSTR("fusion_mode"), &c.fusion.mode, fusionModeChoices),
        Param(PSTR("fusion_gain"), &c.fusion.gain),

        Param(PSTR("input_roll_rate"), &c.input.rate[0]),
        Param(PSTR("input_roll_srate"), &c.input.superRate[0]),
        Param(PSTR("input_roll_expo"), &c.input.expo[0]),

        Param(PSTR("input_pitch_rate"), &c.input.rate[1]),
        Param(PSTR("input_pitch_srate"), &c.input.superRate[1]),
        Param(PSTR("input_pitch_expo"), &c.input.expo[1]),

        Param(PSTR("input_yaw_rate"), &c.input.rate[2]),
        Param(PSTR("input_yaw_srate"), &c.input.superRate[2]),
        Param(PSTR("input_yaw_expo"), &c.input.expo[2]),

        Param(PSTR("input_deadband"), &c.input.deadband),

        Param(PSTR("input_min"), &c.input.minRc),
        Param(PSTR("input_mid"), &c.input.midRc),
        Param(PSTR("input_max"), &c.input.maxRc),

        Param(PSTR("input_interpolation"), &c.input.interpolationMode, interpolChoices),
        Param(PSTR("input_interpolation_interval"), &c.input.interpolationInterval),

        Param(PSTR("input_0"), &c.input.channel[0]),
        Param(PSTR("input_1"), &c.input.channel[1]),
        Param(PSTR("input_2"), &c.input.channel[2]),
        Param(PSTR("input_3"), &c.input.channel[3]),
        Param(PSTR("input_4"), &c.input.channel[4]),
        Param(PSTR("input_5"), &c.input.channel[5]),
        Param(PSTR("input_6"), &c.input.channel[6]),
        Param(PSTR("input_7"), &c.input.channel[7]),

        Param(PSTR("scaler_0"), &c.scaler[0]),
        Param(PSTR("scaler_1"), &c.scaler[1]),
        Param(PSTR("scaler_2"), &c.scaler[2]),

        Param(PSTR("mode_0"), &c.conditions[0]),
        Param(PSTR("mode_1"), &c.conditions[1]),
        Param(PSTR("mode_2"), &c.conditions[2]),
        Param(PSTR("mode_3"), &c.conditions[3]),
        Param(PSTR("mode_4"), &c.conditions[4]),
        Param(PSTR("mode_5"), &c.conditions[5]),
        Param(PSTR("mode_6"), &c.conditions[6]),
        Param(PSTR("mode_7"), &c.conditions[7]),

        Param(PSTR("pid_sync"), &c.loopSync),

        Param(PSTR("pid_roll_p"), &c.pid[PID_ROLL].P),
        Param(PSTR("pid_roll_i"), &c.pid[PID_ROLL].I),
        Param(PSTR("pid_roll_d"), &c.pid[PID_ROLL].D),

        Param(PSTR("pid_pitch_p"), &c.pid[PID_PITCH].P),
        Param(PSTR("pid_pitch_i"), &c.pid[PID_PITCH].I),
        Param(PSTR("pid_pitch_d"), &c.pid[PID_PITCH].D),

        Param(PSTR("pid_yaw_p"), &c.pid[PID_YAW].P),
        Param(PSTR("pid_yaw_i"), &c.pid[PID_YAW].I),
        Param(PSTR("pid_yaw_d"), &c.pid[PID_YAW].D),

        Param(PSTR("pid_level_p"), &c.pid[PID_LEVEL].P),
        Param(PSTR("pid_level_i"), &c.pid[PID_LEVEL].I),
        Param(PSTR("pid_level_d"), &c.pid[PID_LEVEL].D),

        Param(PSTR("pid_level_angle_limit"), &c.angleLimit),
        Param(PSTR("pid_level_rate_limit"), &c.angleRateLimit),
        Param(PSTR("pid_level_lpf_type"), &c.levelPtermFilter.type, filterTypeChoices),
        Param(PSTR("pid_level_lpf_freq"), &c.levelPtermFilter.freq),

        Param(PSTR("pid_yaw_lpf_type"), &c.yawFilter.type, filterTypeChoices),
        Param(PSTR("pid_yaw_lpf_freq"), &c.yawFilter.freq),

        Param(PSTR("pid_dterm_lpf_type"), &c.dtermFilter.type, filterTypeChoices),
        Param(PSTR("pid_dterm_lpf_freq"), &c.dtermFilter.freq),
        Param(PSTR("pid_dterm_lpf2_type"), &c.dtermFilter2.type, filterTypeChoices),
        Param(PSTR("pid_dterm_lpf2_freq"), &c.dtermFilter2.freq),
        Param(PSTR("pid_dterm_notch_freq"), &c.dtermNotchFilter.freq),
        Param(PSTR("pid_dterm_notch_cutoff"), &c.dtermNotchFilter.cutoff),

        Param(PSTR("pid_dterm_weight"), &c.dtermSetpointWeight),
        Param(PSTR("pid_iterm_limit"), &c.itermWindupPointPercent),
        Param(PSTR("pid_iterm_zero"), &c.lowThrottleZeroIterm),
        Param(PSTR("pid_tpa_scale"), &c.tpaScale),
        Param(PSTR("pid_tpa_breakpoint"), &c.tpaBreakpoint),

        Param(PSTR("mixer_sync"), &c.mixerSync),
        Param(PSTR("mixer_type"), &c.mixerType, mixerTypeChoices),
        Param(PSTR("mixer_yaw_reverse"), &c.yawReverse),

        Param(PSTR("output_motor_protocol"), &c.output.protocol, protocolChoices),
        Param(PSTR("output_motor_async"), &c.output.async),
        Param(PSTR("output_motor_rate"), &c.output.rate),
        Param(PSTR("output_servo_rate"), &c.output.servoRate),

        Param(PSTR("output_min_command"), &c.output.minCommand),
        Param(PSTR("output_min_throttle"), &c.output.minThrottle),
        Param(PSTR("output_max_throttle"), &c.output.maxThrottle),
        Param(PSTR("output_dshot_idle"), &c.output.dshotIdle),

        Param(PSTR("output_0"), &c.output.channel[0]),
        Param(PSTR("output_1"), &c.output.channel[1]),
        Param(PSTR("output_2"), &c.output.channel[2]),
        Param(PSTR("output_3"), &c.output.channel[3]),

#if defined(ESP8266)
        Param(PSTR("pin_input_rx"), &c.pin[PIN_INPUT_RX]),
        Param(PSTR("pin_output_0"), &c.pin[PIN_OUTPUT_0]),
        Param(PSTR("pin_output_1"), &c.pin[PIN_OUTPUT_1]),
        Param(PSTR("pin_output_2"), &c.pin[PIN_OUTPUT_2]),
        Param(PSTR("pin_output_3"), &c.pin[PIN_OUTPUT_3]),
        Param(PSTR("pin_buzzer"), &c.pin[PIN_BUZZER]),
        Param(PSTR("pin_i2c_scl"), &c.pin[PIN_I2C_0_SCL]),
        Param(PSTR("pin_i2c_sda"), &c.pin[PIN_I2C_0_SDA]),
        Param(PSTR("pin_input_adc"), &c.pin[PIN_INPUT_ADC_0]),
#endif
#if defined(ESP32)
        Param(PSTR("output_4"), &c.output.channel[4]),
        Param(PSTR("output_5"), &c.output.channel[5]),
        Param(PSTR("output_6"), &c.output.channel[6]),
        Param(PSTR("output_7"), &c.output.channel[7]),

        Param(PSTR("pin_input_rx"), &c.pin[PIN_INPUT_RX]),
        Param(PSTR("pin_output_0"), &c.pin[PIN_OUTPUT_0]),
        Param(PSTR("pin_output_1"), &c.pin[PIN_OUTPUT_1]),
        Param(PSTR("pin_output_2"), &c.pin[PIN_OUTPUT_2]),
        Param(PSTR("pin_output_3"), &c.pin[PIN_OUTPUT_3]),
        Param(PSTR("pin_output_4"), &c.pin[PIN_OUTPUT_4]),
        Param(PSTR("pin_output_5"), &c.pin[PIN_OUTPUT_5]),
        Param(PSTR("pin_output_6"), &c.pin[PIN_OUTPUT_6]),
        Param(PSTR("pin_output_7"), &c.pin[PIN_OUTPUT_7]),
        Param(PSTR("pin_buzzer"), &c.pin[PIN_BUZZER]),
        Param(PSTR("pin_serial_0_tx"), &c.pin[PIN_SERIAL_0_TX]),
        Param(PSTR("pin_serial_0_rx"), &c.pin[PIN_SERIAL_0_RX]),
        Param(PSTR("pin_serial_1_tx"), &c.pin[PIN_SERIAL_1_TX]),
        Param(PSTR("pin_serial_1_rx"), &c.pin[PIN_SERIAL_1_RX]),
        Param(PSTR("pin_serial_2_tx"), &c.pin[PIN_SERIAL_2_TX]),
        Param(PSTR("pin_serial_2_rx"), &c.pin[PIN_SERIAL_2_RX]),
        Param(PSTR("pin_i2c_scl"), &c.pin[PIN_I2C_0_SCL]),
        Param(PSTR("pin_i2c_sda"), &c.pin[PIN_I2C_0_SDA]),
        Param(PSTR("pin_input_adc_0"), &c.pin[PIN_INPUT_ADC_0]),
        Param(PSTR("pin_input_adc_1"), &c.pin[PIN_INPUT_ADC_1]),
        Param(PSTR("pin_spi_0_sck"), &c.pin[PIN_SPI_0_SCK]),
        Param(PSTR("pin_spi_0_mosi"), &c.pin[PIN_SPI_0_MOSI]),
        Param(PSTR("pin_spi_0_miso"), &c.pin[PIN_SPI_0_MISO]),
        Param(PSTR("pin_spi_0_cs_0"), &c.pin[PIN_SPI_0_CS0]),
#endif

        Param(PSTR("pin_buzzer_invert"), &c.buzzer.inverted),

        Param(PSTR("i2c_speed"), &c.i2cSpeed),
        //Param(PSTR("telemetry"), &c.telemetry),
        //Param(PSTR("telemetry_interval"), &c.telemetryInterval),
        //Param(PSTR("soft_serial_guard"), &c.softSerialGuard),
        //Param(PSTR("serial_rx_guard"), &c.serialRxGuard),

        Param(PSTR("wifi_mode"), &c.wireless.mode, wifiModeChoices),
        Param(PSTR("wifi_ssid"), PARAM_STRING, &c.wireless.ssid[0], NULL),
        Param(PSTR("wifi_pass"), PARAM_STRING, &c.wireless.pass[0], NULL),
        Param(PSTR("wifi_ssid_ap"), PARAM_STRING, &c.wireless.ssidAp[0], NULL),
        Param(PSTR("wifi_pass_ap"), PARAM_STRING, &c.wireless.passAp[0], NULL),
        Param(PSTR("wifi_tcp_port"), &c.wireless.port),

        Param(PSTR("mix_outputs"), &c.customMixerCount),
        Param(PSTR("mix_0"), &c.customMixes[i++]),
        Param(PSTR("mix_1"), &c.customMixes[i++]),
        Param(PSTR("mix_2"), &c.customMixes[i++]),
        Param(PSTR("mix_3"), &c.customMixes[i++]),
        Param(PSTR("mix_4"), &c.customMixes[i++]),
        Param(PSTR("mix_5"), &c.customMixes[i++]),
        Param(PSTR("mix_6"), &c.customMixes[i++]),
        Param(PSTR("mix_7"), &c.customMixes[i++]),
        Param(PSTR("mix_8"), &c.customMixes[i++]),
        Param(PSTR("mix_9"), &c.customMixes[i++]),
        Param(PSTR("mix_10"), &c.customMixes[i++]),
        Param(PSTR("mix_11"), &c.customMixes[i++]),
        Param(PSTR("mix_12"), &c.customMixes[i++]),
        Param(PSTR("mix_13"), &c.customMixes[i++]),
        Param(PSTR("mix_14"), &c.customMixes[i++]),
        Param(PSTR("mix_15"), &c.customMixes[i++]),
        Param(PSTR("mix_16"), &c.customMixes[i++]),
        Param(PSTR("mix_17"), &c.customMixes[i++]),
        Param(PSTR("mix_18"), &c.customMixes[i++]),
        Param(PSTR("mix_19"), &c.customMixes[i++]),
        Param(PSTR("mix_20"), &c.customMixes[i++]),
        Param(PSTR("mix_21"), &c.customMixes[i++]),
        Param(PSTR("mix_22"), &c.customMixes[i++]),
        Param(PSTR("mix_23"), &c.customMixes[i++]),
        Param(PSTR("mix_24"), &c.customMixes[i++]),
        Param(PSTR("mix_25"), &c.customMixes[i++]),
        Param(PSTR("mix_26"), &c.customMixes[i++]),
        Param(PSTR("mix_27"), &c.customMixes[i++]),
        Param(PSTR("mix_28"), &c.customMixes[i++]),
        Param(PSTR("mix_29"), &c.customMixes[i++]),
        Param(PSTR("mix_30"), &c.customMixes[i++]),
        Param(PSTR("mix_31"), &c.customMixes[i++]),
        Param(PSTR("mix_32"), &c.customMixes[i++]),
        Param(PSTR("mix_33"), &c.customMixes[i++]),
        Param(PSTR("mix_34"), &c.customMixes[i++]),
        Param(PSTR("mix_35"), &c.customMixes[i++]),
        Param(PSTR("mix_36"), &c.customMixes[i++]),
        Param(PSTR("mix_37"), &c.customMixes[i++]),
        Param(PSTR("mix_38"), &c.customMixes[i++]),
        Param(PSTR("mix_39"), &c.customMixes[i++]),
        Param(PSTR("mix_40"), &c.customMixes[i++]),
        Param(PSTR("mix_41"), &c.customMixes[i++]),
        Param(PSTR("mix_42"), &c.customMixes[i++]),
        Param(PSTR("mix_43"), &c.customMixes[i++]),
        Param(PSTR("mix_44"), &c.customMixes[i++]),
        Param(PSTR("mix_45"), &c.customMixes[i++]),
        Param(PSTR("mix_46"), &c.customMixes[i++]),
        Param(PSTR("mix_47"), &c.customMixes[i++]),
        Param(PSTR("mix_48"), &c.customMixes[i++]),
        Param(PSTR("mix_49"), &c.customMixes[i++]),
        Param(PSTR("mix_50"), &c.customMixes[i++]),
        Param(PSTR("mix_51"), &c.customMixes[i++]),
        Param(PSTR("mix_52"), &c.customMixes[i++]),
        Param(PSTR("mix_53"), &c.customMixes[i++]),
        Param(PSTR("mix_54"), &c.customMixes[i++]),
        Param(PSTR("mix_55"), &c.customMixes[i++]),
        Param(PSTR("mix_56"), &c.customMixes[i++]),
        Param(PSTR("mix_57"), &c.customMixes[i++]),
        Param(PSTR("mix_58"), &c.customMixes[i++]),
        Param(PSTR("mix_59"), &c.customMixes[i++]),
        Param(PSTR("mix_60"), &c.customMixes[i++]),
        Param(PSTR("mix_61"), &c.customMixes[i++]),
        Param(PSTR("mix_62"), &c.customMixes[i++]),
        Param(PSTR("mix_63"), &c.customMixes[i++]),

        Param()
      };
      return params;
    }

    int begin()
    {
      _stream_main = (Stream*)Hardware::getSerialPort(_model.config.serial, SERIAL_FUNCTION_MSP);
      return 1;
    }

    int update()
    {
      if(!_stream_main) return 0;
      return update(_stream_main);
    }

    int update(Stream * s)
    {
      while(s->available() > 0)
      {
        _stream = s; // UGLY, I know
        char c = s->read();
        bool consumed = _msp.process(c, *s);
        if(!consumed)
        {
          process(c);
        }
      }
      return 1;
    }

    bool process(const char c)
    {
      // configurator handshake
      if(!_active && c == '#')
      {
        //FIXME: detect disconnection
        _active = true;
        printVersion();
        println(F(", CLI mode, type help"));
        return true;
      }

      bool endl = c == '\n' || c == '\r';
      if(_index && endl)
      {
        parse();
        execute();
        _index = 0;
        _buff[_index] = '\0';
        return true;
      }

      if(c == '#') _ignore = true;
      else if(endl) _ignore = false;

      // don't put characters into buffer in specific conditions
      if(_ignore || endl || _index >= BUFF_SIZE - 1) return false;

      if(c == '\b') // handle backspace
      {
        _buff[--_index] = '\0';
      }
      else
      {
        _buff[_index] = c;
        _buff[++_index] = '\0';
      }
      return false;
    }

    void parse()
    {
      _cmd = Cmd();
      const char * DELIM = " \t";
      char * pch = std::strtok(_buff, DELIM);
      size_t count = 0;
      while(pch)
      {
        _cmd.args[count++] = pch;
        pch = std::strtok(NULL, DELIM);
      }
    }

    void execute()
    {
      if(_cmd.args[0]) print(F("# "));
      for(size_t i = 0; i < Cmd::ARGS_SIZE; ++i)
      {
        if(!_cmd.args[i]) break;
        print(_cmd.args[i]);
        print(' ');
      }
      println();

      if(!_cmd.args[0]) return;

      if(strcmp_P(_cmd.args[0], PSTR("help")) == 0)
      {
        static const char * helps[] = {
          PSTR("available commands:"),
          PSTR(" help"), PSTR(" dump"), PSTR(" get param"), PSTR(" set param value ..."), PSTR(" cal [gyro]"),
          PSTR(" defaults"), PSTR(" save"), PSTR(" reboot"), PSTR(" scaler"), PSTR(" mixer"),
          PSTR(" stats"), PSTR(" status"), PSTR(" devinfo"), PSTR(" version"),
          //PSTR(" load"), PSTR(" eeprom"),
          //PSTR(" fsinfo"), PSTR(" fsformat"), PSTR(" logs"),  PSTR(" log"),
          NULL
        };
        for(const char ** ptr = helps; *ptr; ptr++) {
          println(FPSTR(*ptr));
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("version")) == 0)
      {
        printVersion();
        println();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("wifi")) == 0)
      {
        print(F("tcp://"));
        print(_model.state.localIp);
        print(F(":"));
        println(_model.config.wireless.port);
      }
      #if defined(ESP32)
      else if(strcmp_P(_cmd.args[0], PSTR("tasks")) == 0)
      {
        printVersion();
        println();

        size_t numTasks = uxTaskGetNumberOfTasks();

        print(F("num tasks: "));
        print(numTasks);
        println();
      }
      #endif
      else if(strcmp_P(_cmd.args[0], PSTR("devinfo")) == 0)
      {
        printVersion();
        println();
        print(F("  bool: ")); println(sizeof(bool));
        print(F("  char: ")); println(sizeof(char));
        print(F(" short: ")); println(sizeof(short));
        print(F("   int: ")); println(sizeof(int));
        print(F("  long: ")); println(sizeof(long));
        print(F(" float: ")); println(sizeof(float));
        print(F("double: ")); println(sizeof(double));
        print(F(" model: ")); println(sizeof(ModelConfig));
        println();

#if defined(ESP8266)
        const rst_info * resetInfo = system_get_rst_info();
        print(F("reset reason: "));
        println(resetInfo->reason);

        print(F("free heap: "));
        println(ESP.getFreeHeap());

        print(F("os print: "));
        println(system_get_os_print());

        //system_print_meminfo();

        print(F("chip id: 0x"));
        println(system_get_chip_id(), HEX);

        print(F("sdk version: "));
        println(system_get_sdk_version());

        print(F("boot version: "));
        println(system_get_boot_version());

        print(F("userbin addr: 0x"));
        println(system_get_userbin_addr(), HEX);

        print(F("boot mode: "));
        println(system_get_boot_mode() == 0 ? F("SYS_BOOT_ENHANCE_MODE") : F("SYS_BOOT_NORMAL_MODE"));

        print(F("cpu freq: "));
        println(ESP.getCpuFreqMHz());

        print(F("flash size map: "));
        println(system_get_flash_size_map());

        print(F("time: "));
        println(system_get_time() / 1000000);
#endif
      }
      else if(strcmp_P(_cmd.args[0], PSTR("get")) == 0)
      {
        bool found = false;
        for(size_t i = 0; _params[i].name; ++i)
        {
          String ts = FPSTR(_params[i].name);
          if(!_cmd.args[1] || ts.indexOf(_cmd.args[1]) >= 0)
          {
            print(_params[i]);
            found = true;
          }
        }
        if(!found)
        {
          print(F("param not found: "));
          print(_cmd.args[1]);
        }
        println();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("set")) == 0)
      {
        if(!_cmd.args[1])
        {
          println(F("param required"));
          println();
          return;
        }
        bool found = false;
        for(size_t i = 0; _params[i].name; ++i)
        {
          if(strcmp_P(_cmd.args[1], _params[i].name) == 0)
          {
            _params[i].update(_cmd.args);
            print(_params[i]);
            found = true;
            break;
          }
        }
        if(!found)
        {
          print(F("param not found: "));
          println(_cmd.args[1]);
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("dump")) == 0)
      {
        print('#');
        printVersion();
        println();
        for(size_t i = 0; _params[i].name; ++i)
        {
          print(_params[i]);
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("cal")) == 0)
      {
        if(!_cmd.args[1])
        {
          print(F(" gyro offset: "));
          print(_model.config.gyroBias[0]); print(' ');
          print(_model.config.gyroBias[1]); print(' ');
          print(_model.config.gyroBias[2]); print(F(" ["));
          print(_model.state.gyroBias[0]); print(' ');
          print(_model.state.gyroBias[1]); print(' ');
          print(_model.state.gyroBias[2]); println(F("]"));

          print(F("accel offset: "));
          print(_model.config.accelBias[0]); print(' ');
          print(_model.config.accelBias[1]); print(' ');
          print(_model.config.accelBias[2]); print(F(" ["));
          print(_model.state.accelBias[0]); print(' ');
          print(_model.state.accelBias[1]); print(' ');
          print(_model.state.accelBias[2]); println(F("]"));

          /*
          print(F("mag offset: "));
          print(_model.config.magCalibrationOffset[0]); print(' ');
          print(_model.config.magCalibrationOffset[1]); print(' ');
          print(_model.config.magCalibrationOffset[2]); print(F(" ["));
          print(_model.state.magCalibrationOffset[0]); print(' ');
          print(_model.state.magCalibrationOffset[1]); print(' ');
          print(_model.state.magCalibrationOffset[2]); println(F("]"));

          print(F(" mag scale: "));
          print(_model.config.magCalibrationScale[0]); print(' ');
          print(_model.config.magCalibrationScale[1]); print(' ');
          print(_model.config.magCalibrationScale[2]); print(F(" ["));
          print(_model.state.magCalibrationScale[0]); print(' ');
          print(_model.state.magCalibrationScale[1]); print(' ');
          print(_model.state.magCalibrationScale[2]); println(F("]"));
          */
        }
        else if(strcmp_P(_cmd.args[1], PSTR("gyro")) == 0)
        {
          if(!_model.isActive(MODE_ARMED)) _model.calibrate();
          println(F("OK"));
        }
        else if(strcmp_P(_cmd.args[1], PSTR("mag")) == 0)
        {
          if(!_cmd.args[2]) {}
          else if(_cmd.args[2][0] == '1')
          {
            _model.state.magCalibration = 1;
            //_model.config.telemetry = 1;
            //_model.config.telemetryInterval = 200;
            print(F("mag calibration on"));
          }
          else if(_cmd.args[2][0] == '0')
          {
            _model.state.magCalibration = 0;
            //_model.config.telemetry = 0;
            print(F("mag calibration off"));
          }
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("preset")) == 0)
      {
        if(!_cmd.args[1])
        {
          println(F("Available presets: scaler, modes, micrus, brobot"));
        }
        else if(strcmp_P(_cmd.args[1], PSTR("scaler")) == 0)
        {
          _model.config.scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
          _model.config.scaler[0].channel = 5;
          _model.config.scaler[0].minScale = 25; //%
          _model.config.scaler[0].maxScale = 400;

          _model.config.scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
          _model.config.scaler[1].channel = 6;
          _model.config.scaler[1].minScale = 25; //%
          _model.config.scaler[1].maxScale = 400;

          _model.config.scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
          _model.config.scaler[2].channel = 7;
          _model.config.scaler[2].minScale = 25; //%
          _model.config.scaler[2].maxScale = 400;

          println(F("OK"));
        }
        else if(strcmp_P(_cmd.args[1], PSTR("modes")) == 0)
        {
          _model.config.conditions[0].id = MODE_ARMED;
          _model.config.conditions[0].ch = AXIS_AUX_1 + 0;
          _model.config.conditions[0].min = 1700;
          _model.config.conditions[0].max = 2100;

          _model.config.conditions[1].id = MODE_ANGLE;
          _model.config.conditions[1].ch = AXIS_AUX_1 + 0; // aux1
          _model.config.conditions[1].min = 1900;
          _model.config.conditions[1].max = 2100;

          _model.config.conditions[2].id = MODE_AIRMODE;
          _model.config.conditions[2].ch = 0; // aux1
          _model.config.conditions[2].min = (1700 - 900) / 25;
          _model.config.conditions[2].max = (2100 - 900) / 25;

          println(F("OK"));
        }
        else if(strcmp_P(_cmd.args[1], PSTR("micrus")) == 0)
        {
          println(F("OK"));
        }
        else if(strcmp_P(_cmd.args[1], PSTR("brobot")) == 0)
        {
          println(F("OK"));
        }
        else
        {
          println(F("NOT OK"));
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("load")) == 0)
      {
        _model.load();
        println(F("OK"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("save")) == 0)
      {
        _model.save();
        println(F("OK"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("eeprom")) == 0)
      {
        int start = 0;
        if(_cmd.args[1])
        {
          start = std::max(String(_cmd.args[1]).toInt(), 0L);
        }

        for(int i = start; i < start + 32; ++i)
        {
          uint8_t v = EEPROM.read(i);
          if(v <= 0xf) print('0');
          print(v, HEX);
          print(' ');
        }
        println();

        for(int i = start; i < start + 32; ++i)
        {
          print((int8_t)EEPROM.read(i));
          print(' ');
        }
        println();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("scaler")) == 0)
      {
        for(size_t i = 0; i < SCALER_COUNT; i++)
        {
          uint32_t mode = _model.config.scaler[i].dimension;
          if(!mode) continue;
          short c = _model.config.scaler[i].channel;
          float v = _model.state.input[c];
          float min = _model.config.scaler[i].minScale * 0.01f;
          float max = _model.config.scaler[i].maxScale * 0.01f;
          float scale = Math::map3(v, -1.f, 0.f, 1.f, min, min < 0 ? 0.f : 1.f, max);
          print(F("scaler: "));
          print(i);
          print(' ');
          print(mode);
          print(' ');
          print(min);
          print(' ');
          print(max);
          print(' ');
          print(v);
          print(' ');
          println(scale);
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("mixer")) == 0)
      {
        const MixerConfig& mixer = _model.state.currentMixer;
        print(F("set mix_outputs "));
        println(mixer.count);
        Param p;
        for(size_t i = 0; i < MIXER_RULE_MAX; i++)
        {
          print(F("set mix_"));
          print(i);
          print(' ');
          p.print(*_stream, mixer.mixes[i]);
          println();
          if(mixer.mixes[i].src == MIXER_SOURCE_NULL) break;
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("status")) == 0)
      {
        const char ** busNames = Device::BusDevice::getNames();
        const char ** gyroNames = Device::GyroDevice::getNames();

        printVersion();
        println();
        print(F("STATUS: "));
        println();
        print(F(" cpu freq : "));
        println(ESP.getCpuFreqMHz());
        print(F(" gyro bus : "));
        println(FPSTR(busNames[_model.state.gyroBus]));
        print(F(" gyro type: "));
        println(FPSTR(gyroNames[_model.state.gyroDev]));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("stats")) == 0)
      {
        printVersion();
        println();
        print(F("    cpu freq: "));
        print(ESP.getCpuFreqMHz());
        println(F(" MHz"));
        print(F("   gyro rate: "));
        print(_model.state.gyroTimer.rate);
        println(F(" Hz"));
        print(F("   loop rate: "));
        print(_model.state.loopTimer.rate);
        println(F(" Hz"));
        print(F("  mixer rate: "));
        print(_model.state.mixerTimer.rate);
        println(F(" Hz"));
        println();
        for(size_t i = 0; i < COUNTER_COUNT; ++i)
        {
          print(FPSTR(_model.state.stats.getName((StatCounter)i)));
          print(": ");
          print((int)(_model.state.stats.getTime((StatCounter)i) * _model.state.loopTimer.interval), 1);
          print("us, ");
          print(_model.state.stats.getLoad((StatCounter)i), 1);
          print("%");
          println();
        }
        print(F("  TOTAL: "));
        print((int)(_model.state.stats.getTotalTime() * _model.state.loopTimer.interval));
        print(F("us, "));
        print(_model.state.stats.getTotalLoad(), 1);
        print(F("%"));
        println();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("fsinfo")) == 0)
      {
        _model.logger.info(_stream);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("fsformat")) == 0)
      {
        print(F("wait... "));
        _model.logger.format();
        println(F("OK"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("logs")) == 0)
      {
        _model.logger.list(_stream);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("log")) == 0)
      {
        if(!_cmd.args[1])
        {
          _model.logger.show(_stream);
          return;
        }
        int id = String(_cmd.args[1]).toInt();
        if(!id)
        {
          println(F("invalid log id"));
          println();
          return;
        }
        _model.logger.show(_stream, id);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("reboot")) == 0)
      {
        Hardware::restart(_model);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("defaults")) == 0)
      {
        _model.reset();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("exit")) == 0)
      {
        _active = false;
      }
      else
      {
        print(F("unknown command: "));
        println(_cmd.args[0]);
      }
      println();
    }

  private:
    template<typename T>
    void print(const T& t)
    {
      if(!_stream) return;
      (*_stream).print(t);
    }

    template<typename T, typename V>
    void print(const T& t, const V& v)
    {
      if(!_stream) return;
      (*_stream).print(t, v);
    }

    template<typename T>
    void println(const T& t)
    {
      if(!_stream) return;
      (*_stream).println(t);
    }

    template<typename T, typename V>
    void println(const T& t, const V& v)
    {
      if(!_stream) return;
      (*_stream).println(t, v);
    }

    void println()
    {
      if(!_stream) return;
      (*_stream).println();
    }

    void print(const Param& param)
    {
      if(!_stream) return;
      print(F("set "));
      print(FPSTR(param.name));
      print(' ');
      param.print(*_stream);
      println();
    }

    void printVersion()
    {
      print(TARGET_BOARD_IDENTIFIER);
      print(' ');
      print(targetName);
      print(' ');
      print('v');
      print(targetVersion);
      print(' ');
      print(shortGitRevision);
      print(' ');
      print(buildDate);
      print(' ');
      print(buildTime);
    }

    static const size_t BUFF_SIZE = 64;

    Model& _model;
    Stream * _stream;
    Stream * _stream_main;
    const Param * _params;
    char _buff[BUFF_SIZE];
    size_t _index;
    Cmd _cmd;
    Msp _msp;
    bool _ignore;
    bool _active;
};

}

#endif
