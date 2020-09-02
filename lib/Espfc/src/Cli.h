#ifndef _ESPFC_CLI_H_
#define _ESPFC_CLI_H_

#include <Arduino.h>
#include <algorithm>

#include "Model.h"
#include "Hardware.h"
#include "Logger.h"
#include "Device/GyroDevice.h"
#include "platform.h"

#if defined(ESP8266)
#include "user_interface.h"
#endif //ESP8266

#if defined(ESP32)
#include <freertos/task.h>
#endif

namespace Espfc {

class Cli
{
  public:
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
      PARAM_SERIAL,  // mixer config
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
        Param(const char * n, SerialPortConfig * a):    Param(n, PARAM_SERIAL, reinterpret_cast<char*>(a), NULL) {}

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
            case PARAM_SERIAL:
              print(stream, *reinterpret_cast<SerialPortConfig*>(addr));
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
          stream.print(' ');
          stream.print(ac.logicMode);
          stream.print(' ');
          stream.print(ac.linkId);
        }

        void print(Stream& stream, const MixerEntry& me) const
        {
          stream.print(me.src);
          stream.print(' ');
          stream.print(me.dst);
          stream.print(' ');
          stream.print(me.rate);
        }

        void print(Stream& stream, const SerialPortConfig& sc) const
        {
          stream.print(sc.functionMask);
          stream.print(' ');
          stream.print(sc.baud);
          stream.print(' ');
          stream.print(sc.blackboxBaud);
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
            case PARAM_SERIAL:
              write(*reinterpret_cast<SerialPortConfig*>(addr), args);
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
          if(args[6]) ac.max = String(args[6]).toInt();
          if(args[7]) ac.max = String(args[7]).toInt();
        }

        void write(MixerEntry& ac, const char ** args) const
        {
          if(args[2]) ac.src = constrain(String(args[2]).toInt(), 0, MIXER_SOURCE_MAX - 1);
          if(args[3]) ac.dst = constrain(String(args[3]).toInt(), 0, (int)(OUTPUT_CHANNELS - 1));
          if(args[4]) ac.rate = constrain(String(args[4]).toInt(), -1000, 1000);
        }

        void write(SerialPortConfig& sc, const char ** args) const
        {
          if(args[2]) sc.functionMask = String(args[2]).toInt();
          if(args[3]) sc.baud = String(args[3]).toInt();
          if(args[4]) sc.blackboxBaud = String(args[4]).toInt();
        }

        template<typename T>
        void write(const T v) const
        {
          *reinterpret_cast<T*>(addr) = v;
        }

        void write(const String& v) const
        {
          *addr = 0;
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

    Cli(Model& model): _model(model), _ignore(false), _active(false)
    {
      _params = initialize(_model.config);
    }

    static const Param * initialize(ModelConfig& c)
    {
      const char ** busDevChoices            = Device::BusDevice::getNames();
      const char ** gyroDevChoices           = Device::GyroDevice::getNames();
      const char ** baroDevChoices           = Device::BaroDevice::getNames();
      const char ** magDevChoices            = Device::MagDevice::getNames();

      const char ** fusionModeChoices        = FusionConfig::getModeNames();
      static const char* gyroDlpfChoices[]   = { PSTR("256Hz"), PSTR("188Hz"), PSTR("98Hz"), PSTR("42Hz"), PSTR("20Hz"), PSTR("10Hz"), PSTR("5Hz"), PSTR("EXPERIMENTAL"), NULL };
      static const char* debugModeChoices[]  = {  PSTR("NONE"),   PSTR("CYCLETIME"), PSTR("BATTERY"), PSTR("GYRO_FILTERED"), PSTR("ACCELEROMETER"), PSTR("PIDLOOP"), PSTR("GYRO_SCALED"), PSTR("RC_INTERPOLATION"), 
                                                  PSTR("ANGLERATE"), PSTR("ESC_SENSOR"), PSTR("SCHEDULER"), PSTR("STACK"), PSTR("ESC_SENSOR_RPM"), PSTR("ESC_SENSOR_TMP"), PSTR("ALTITUDE"), PSTR("FFT"), 
                                                  PSTR("FFT_TIME"), PSTR("FFT_FREQ"), PSTR("RX_FRSKY_SPI"), PSTR("RX_SFHSS_SPI"), PSTR("GYRO_RAW"), PSTR("DUAL_GYRO_RAW"), PSTR("DUAL_GYRO_DIFF"), 
                                                  PSTR("MAX7456_SIGNAL"), PSTR("MAX7456_SPICLOCK"), PSTR("SBUS"), PSTR("FPORT"), PSTR("RANGEFINDER"), PSTR("RANGEFINDER_QUALITY"), PSTR("LIDAR_TF"), 
                                                  PSTR("ADC_INTERNAL"), PSTR("RUNAWAY_TAKEOFF"), PSTR("SDIO"), PSTR("CURRENT_SENSOR"), PSTR("USB"), PSTR("SMARTAUDIO"), PSTR("RTH"), PSTR("ITERM_RELAX"), 
                                                  PSTR("ACRO_TRAINER"), PSTR("RC_SMOOTHING"), PSTR("RX_SIGNAL_LOSS"), PSTR("RC_SMOOTHING_RATE"), PSTR("ANTI_GRAVITY"), PSTR("DYN_LPF"), PSTR("RX_SPEKTRUM_SPI"), 
                                                  PSTR("DSHOT_RPM_TELEMETRY"), PSTR("RPM_FILTER"), PSTR("D_MIN"), PSTR("AC_CORRECTION"), PSTR("AC_ERROR"), PSTR("DUAL_GYRO_SCALED"), PSTR("DSHOT_RPM_ERRORS"), 
                                                  PSTR("CRSF_LINK_STATISTICS_UPLINK"), PSTR("CRSF_LINK_STATISTICS_PWR"), PSTR("CRSF_LINK_STATISTICS_DOWN"), PSTR("BARO"), PSTR("GPS_RESCUE_THROTTLE_PID"), 
                                                  PSTR("DYN_IDLE"), PSTR("FF_LIMIT"), PSTR("FF_INTERPOLATED"), PSTR("BLACKBOX_OUTPUT"), PSTR("GYRO_SAMPLE"), PSTR("RX_TIMING"), NULL };
      static const char* filterTypeChoices[] = { PSTR("PT1"), PSTR("BIQUAD"), PSTR("NOTCH"), PSTR("NOTCH_DF1"), PSTR("BPF"), PSTR("FIR2"), PSTR("MEDIAN3"), PSTR("NONE"), NULL };
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
      static const char* inputRateTypeChoices[] = { PSTR("BETAFLIGHT"), PSTR("RACEFLIGHT"), PSTR("KISS"), PSTR("ACTUAL"), PSTR("QUICK"), NULL };
      static const char* throtleLimitTypeChoices[] = { PSTR("NONE"), PSTR("SCALE"), PSTR("CLIP"), NULL };

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
        Param(PSTR("gyro_dyn_lpf_min"), &c.gyroDynLpfFilter.cutoff),
        Param(PSTR("gyro_dyn_lpf_max"), &c.gyroDynLpfFilter.freq),
        Param(PSTR("gyro_offset_x"), &c.gyroBias[0]),
        Param(PSTR("gyro_offset_y"), &c.gyroBias[1]),
        Param(PSTR("gyro_offset_z"), &c.gyroBias[2]),

        Param(PSTR("accel_bus"), &c.accelBus, busDevChoices),
        Param(PSTR("accel_dev"), &c.accelDev, gyroDevChoices),
        Param(PSTR("accel_align"), &c.accelAlign, alignChoices),
        Param(PSTR("accel_lpf_type"), &c.accelFilter.type, filterTypeChoices),
        Param(PSTR("accel_lpf_freq"), &c.accelFilter.freq),
        Param(PSTR("accel_offset_x"), &c.accelBias[0]),
        Param(PSTR("accel_offset_y"), &c.accelBias[1]),
        Param(PSTR("accel_offset_z"), &c.accelBias[2]),

        
        Param(PSTR("mag_bus"), &c.magBus, busDevChoices),
        Param(PSTR("mag_dev"), &c.magDev, magDevChoices),
        Param(PSTR("mag_align"), &c.magAlign, alignChoices),
        Param(PSTR("mag_filter_type"), &c.magFilter.type, filterTypeChoices),
        Param(PSTR("mag_filter_lpf"), &c.magFilter.freq),
        Param(PSTR("mag_offset_x"), &c.magCalibrationOffset[0]),
        Param(PSTR("mag_offset_y"), &c.magCalibrationOffset[1]),
        Param(PSTR("mag_offset_z"), &c.magCalibrationOffset[2]),
        Param(PSTR("mag_scale_x"), &c.magCalibrationScale[0]),
        Param(PSTR("mag_scale_y"), &c.magCalibrationScale[1]),
        Param(PSTR("mag_scale_z"), &c.magCalibrationScale[2]),
        

        Param(PSTR("baro_bus"), &c.baroBus, busDevChoices),
        Param(PSTR("baro_dev"), &c.baroDev, baroDevChoices),
        Param(PSTR("baro_lpf_type"), &c.baroFilter.type, filterTypeChoices),
        Param(PSTR("baro_lpf_freq"), &c.baroFilter.freq),

        Param(PSTR("fusion_mode"), &c.fusion.mode, fusionModeChoices),
        Param(PSTR("fusion_gain"), &c.fusion.gain),

        Param(PSTR("input_rate_type"), &c.input.rateType, inputRateTypeChoices),

        Param(PSTR("input_roll_rate"), &c.input.rate[0]),
        Param(PSTR("input_roll_srate"), &c.input.superRate[0]),
        Param(PSTR("input_roll_expo"), &c.input.expo[0]),
        Param(PSTR("input_roll_limit"), &c.input.rateLimit[0]),

        Param(PSTR("input_pitch_rate"), &c.input.rate[1]),
        Param(PSTR("input_pitch_srate"), &c.input.superRate[1]),
        Param(PSTR("input_pitch_expo"), &c.input.expo[1]),
        Param(PSTR("input_pitch_limit"), &c.input.rateLimit[1]),

        Param(PSTR("input_yaw_rate"), &c.input.rate[2]),
        Param(PSTR("input_yaw_srate"), &c.input.superRate[2]),
        Param(PSTR("input_yaw_expo"), &c.input.expo[2]),
        Param(PSTR("input_yaw_limit"), &c.input.rateLimit[2]),

        Param(PSTR("input_deadband"), &c.input.deadband),

        Param(PSTR("input_min"), &c.input.minRc),
        Param(PSTR("input_mid"), &c.input.midRc),
        Param(PSTR("input_max"), &c.input.maxRc),

        Param(PSTR("input_interpolation"), &c.input.interpolationMode, interpolChoices),
        Param(PSTR("input_interpolation_interval"), &c.input.interpolationInterval),

        Param(PSTR("input_rssi_channel"), &c.input.rssiChannel),

        Param(PSTR("input_0"), &c.input.channel[0]),
        Param(PSTR("input_1"), &c.input.channel[1]),
        Param(PSTR("input_2"), &c.input.channel[2]),
        Param(PSTR("input_3"), &c.input.channel[3]),
        Param(PSTR("input_4"), &c.input.channel[4]),
        Param(PSTR("input_5"), &c.input.channel[5]),
        Param(PSTR("input_6"), &c.input.channel[6]),
        Param(PSTR("input_7"), &c.input.channel[7]),
        Param(PSTR("input_8"), &c.input.channel[8]),
        Param(PSTR("input_9"), &c.input.channel[9]),
        Param(PSTR("input_10"), &c.input.channel[10]),
        Param(PSTR("input_11"), &c.input.channel[11]),
        Param(PSTR("input_12"), &c.input.channel[12]),
        Param(PSTR("input_13"), &c.input.channel[13]),
        Param(PSTR("input_14"), &c.input.channel[14]),
        Param(PSTR("input_15"), &c.input.channel[15]),

        Param(PSTR("serial_0"), &c.serial[SERIAL_UART_0]),
        Param(PSTR("serial_1"), &c.serial[SERIAL_UART_1]),
#if defined(ESP32)
        Param(PSTR("serial_2"), &c.serial[SERIAL_UART_2]),
        Param(PSTR("serial_soft_0"), &c.serial[SERIAL_SOFT_0]),
#elif defined(USE_SOFT_SERIAL)
        Param(PSTR("serial_soft_0"), &c.serial[SERIAL_SOFT_0]),
#endif

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
        Param(PSTR("pid_roll_f"), &c.pid[PID_ROLL].F),

        Param(PSTR("pid_pitch_p"), &c.pid[PID_PITCH].P),
        Param(PSTR("pid_pitch_i"), &c.pid[PID_PITCH].I),
        Param(PSTR("pid_pitch_d"), &c.pid[PID_PITCH].D),
        Param(PSTR("pid_pitch_f"), &c.pid[PID_PITCH].F),

        Param(PSTR("pid_yaw_p"), &c.pid[PID_YAW].P),
        Param(PSTR("pid_yaw_i"), &c.pid[PID_YAW].I),
        Param(PSTR("pid_yaw_d"), &c.pid[PID_YAW].D),
        Param(PSTR("pid_yaw_f"), &c.pid[PID_YAW].F),

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
        Param(PSTR("pid_dterm_dyn_lpf_min"), &c.dtermDynLpfFilter.cutoff),
        Param(PSTR("pid_dterm_dyn_lpf_max"), &c.dtermDynLpfFilter.freq),

        Param(PSTR("pid_dterm_weight"), &c.dtermSetpointWeight),
        Param(PSTR("pid_iterm_limit"), &c.itermWindupPointPercent),
        Param(PSTR("pid_iterm_zero"), &c.lowThrottleZeroIterm),
        Param(PSTR("pid_tpa_scale"), &c.tpaScale),
        Param(PSTR("pid_tpa_breakpoint"), &c.tpaBreakpoint),

        Param(PSTR("mixer_sync"), &c.mixerSync),
        Param(PSTR("mixer_type"), &c.mixerType, mixerTypeChoices),
        Param(PSTR("mixer_yaw_reverse"), &c.yawReverse),
        Param(PSTR("mixer_throttle_limit_type"), &c.output.throttleLimitType, throtleLimitTypeChoices),
        Param(PSTR("mixer_throttle_limit_percent"), &c.output.throttleLimitPercent),
        Param(PSTR("mixer_output_limit"), &c.output.motorLimit),

        Param(PSTR("output_motor_protocol"), &c.output.protocol, protocolChoices),
        Param(PSTR("output_motor_async"), &c.output.async),
        Param(PSTR("output_motor_rate"), &c.output.rate),
        Param(PSTR("output_servo_rate"), &c.output.servoRate),

        Param(PSTR("output_min_command"), &c.output.minCommand),
        Param(PSTR("output_min_throttle"), &c.output.minThrottle),
        Param(PSTR("output_max_throttle"), &c.output.maxThrottle),
        Param(PSTR("output_dshot_idle"), &c.output.dshotIdle),

#if defined(ESP8266)
        Param(PSTR("output_0"), &c.output.channel[0]),
        Param(PSTR("output_1"), &c.output.channel[1]),
        Param(PSTR("output_2"), &c.output.channel[2]),
        Param(PSTR("output_3"), &c.output.channel[3]),

        Param(PSTR("pin_input_rx"), &c.pin[PIN_INPUT_RX]),
        Param(PSTR("pin_output_0"), &c.pin[PIN_OUTPUT_0]),
        Param(PSTR("pin_output_1"), &c.pin[PIN_OUTPUT_1]),
        Param(PSTR("pin_output_2"), &c.pin[PIN_OUTPUT_2]),
        Param(PSTR("pin_output_3"), &c.pin[PIN_OUTPUT_3]),
        Param(PSTR("pin_buzzer"), &c.pin[PIN_BUZZER]),
        Param(PSTR("pin_i2c_scl"), &c.pin[PIN_I2C_0_SCL]),
        Param(PSTR("pin_i2c_sda"), &c.pin[PIN_I2C_0_SDA]),
        Param(PSTR("pin_input_adc"), &c.pin[PIN_INPUT_ADC_0]),
        Param(PSTR("pin_buzzer_invert"), &c.buzzer.inverted),
#elif defined(ESP32)
        Param(PSTR("output_0"), &c.output.channel[0]),
        Param(PSTR("output_1"), &c.output.channel[1]),
        Param(PSTR("output_2"), &c.output.channel[2]),
        Param(PSTR("output_3"), &c.output.channel[3]),
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
        Param(PSTR("pin_spi_cs_0"), &c.pin[PIN_SPI_CS0]),
        Param(PSTR("pin_spi_cs_1"), &c.pin[PIN_SPI_CS1]),
        Param(PSTR("pin_spi_cs_2"), &c.pin[PIN_SPI_CS2]),
        Param(PSTR("pin_buzzer_invert"), &c.buzzer.inverted),
#endif

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

    bool process(const char c, CliCmd& cmd, Stream& stream)
    {
      // configurator handshake
      if(!_active && c == '#')
      {
        //FIXME: detect disconnection
        _active = true;
        printVersion(stream);
        stream.println(F(", CLI mode, type help"));
        cmd = CliCmd();
        return true;
      }

      bool endl = c == '\n' || c == '\r';
      if(cmd.index && endl)
      {
        parse(cmd);
        execute(cmd, stream);
        //cmd.index = 0;
        //cmd.buff[cmd.index] = '\0';
        cmd = CliCmd();
        return true;
      }

      if(c == '#') _ignore = true;
      else if(endl) _ignore = false;

      // don't put characters into buffer in specific conditions
      if(_ignore || endl || cmd.index >= CLI_BUFF_SIZE - 1) return false;

      if(c == '\b') // handle backspace
      {
        cmd.buff[--cmd.index] = '\0';
      }
      else
      {
        cmd.buff[cmd.index] = c;
        cmd.buff[++cmd.index] = '\0';
      }
      return false;
    }

    void parse(CliCmd& cmd)
    {
      const char * DELIM = " \t";
      char * pch = strtok(cmd.buff, DELIM);
      size_t count = 0;
      while(pch)
      {
        cmd.args[count++] = pch;
        pch = strtok(NULL, DELIM);
      }
    }

    void execute(CliCmd& cmd, Stream& s)
    {
      if(cmd.args[0]) s.print(F("# "));
      for(size_t i = 0; i < CLI_ARGS_SIZE; ++i)
      {
        if(!cmd.args[i]) break;
        s.print(cmd.args[i]);
        s.print(' ');
      }
      s.println();

      if(!cmd.args[0]) return;

      if(strcmp_P(cmd.args[0], PSTR("help")) == 0)
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
        for(const char ** ptr = helps; *ptr; ptr++)
        {
          s.println(FPSTR(*ptr));
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("version")) == 0)
      {
        printVersion(s);
        s.println();
      }
      else if(strcmp_P(cmd.args[0], PSTR("wifi")) == 0)
      {
        s.print(F("IPv4  : tcp://"));
        s.print(_model.state.localIp);
        s.print(F(":"));
        s.println(_model.config.wireless.port);
      }
      #if defined(ESP32)
      else if(strcmp_P(cmd.args[0], PSTR("tasks")) == 0)
      {
        printVersion(s);
        s.println();

        size_t numTasks = uxTaskGetNumberOfTasks();

        s.print(F("num tasks: "));
        s.print(numTasks);
        s.println();
      }
      #endif
      else if(strcmp_P(cmd.args[0], PSTR("devinfo")) == 0)
      {
        printVersion(s);
        s.println();
        /*s.print(F("  bool: ")); s.println(sizeof(bool));
        s.print(F("  char: ")); s.println(sizeof(char));
        s.print(F(" short: ")); s.println(sizeof(short));
        s.print(F("   int: ")); s.println(sizeof(int));
        s.print(F("  long: ")); s.println(sizeof(long));
        s.print(F(" float: ")); s.println(sizeof(float));
        s.print(F("double: ")); s.println(sizeof(double));*/

        s.print(F("config size: "));
        s.println(sizeof(ModelConfig));

        s.print(F("  free heap: "));
        s.println(ESP.getFreeHeap());

        s.print(F("   cpu freq: "));
        s.print(ESP.getCpuFreqMHz());
        s.println(F(" MHz"));

#if defined(ESP32)

#elif defined(ESP8266)
        const rst_info * resetInfo = system_get_rst_info();
        s.print(F("reset reason: "));
        s.println(resetInfo->reason);


        s.print(F("os s.print: "));
        s.println(system_get_os_print());

        //system_print_meminfo();

        s.print(F("chip id: 0x"));
        s.println(system_get_chip_id(), HEX);

        s.print(F("sdk version: "));
        s.println(system_get_sdk_version());

        s.print(F("boot version: "));
        s.println(system_get_boot_version());

        s.print(F("userbin addr: 0x"));
        s.println(system_get_userbin_addr(), HEX);

        s.print(F("boot mode: "));
        s.println(system_get_boot_mode() == 0 ? F("SYS_BOOT_ENHANCE_MODE") : F("SYS_BOOT_NORMAL_MODE"));

        s.print(F("flash size map: "));
        s.println(system_get_flash_size_map());

        s.print(F("time: "));
        s.println(system_get_time() / 1000000);
#endif
      }
      else if(strcmp_P(cmd.args[0], PSTR("get")) == 0)
      {
        bool found = false;
        for(size_t i = 0; _params[i].name; ++i)
        {
          String ts = FPSTR(_params[i].name);
          if(!cmd.args[1] || ts.indexOf(cmd.args[1]) >= 0)
          {
            print(_params[i], s);
            found = true;
          }
        }
        if(!found)
        {
          s.print(F("param not found: "));
          s.print(cmd.args[1]);
        }
        s.println();
      }
      else if(strcmp_P(cmd.args[0], PSTR("set")) == 0)
      {
        if(!cmd.args[1])
        {
          s.println(F("param required"));
          s.println();
          return;
        }
        bool found = false;
        for(size_t i = 0; _params[i].name; ++i)
        {
          if(strcmp_P(cmd.args[1], _params[i].name) == 0)
          {
            _params[i].update(cmd.args);
            print(_params[i], s);
            found = true;
            break;
          }
        }
        if(!found)
        {
          s.print(F("param not found: "));
          s.println(cmd.args[1]);
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("dump")) == 0)
      {
        s.print('#');
        printVersion(s);
        s.println();
        for(size_t i = 0; _params[i].name; ++i)
        {
          print(_params[i], s);
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("cal")) == 0)
      {
        if(!cmd.args[1])
        {
          s.print(F(" gyro offset: "));
          s.print(_model.config.gyroBias[0]); s.print(' ');
          s.print(_model.config.gyroBias[1]); s.print(' ');
          s.print(_model.config.gyroBias[2]); s.print(F(" ["));
          s.print(_model.state.gyroBias[0]); s.print(' ');
          s.print(_model.state.gyroBias[1]); s.print(' ');
          s.print(_model.state.gyroBias[2]); s.println(F("]"));

          s.print(F("accel offset: "));
          s.print(_model.config.accelBias[0]); s.print(' ');
          s.print(_model.config.accelBias[1]); s.print(' ');
          s.print(_model.config.accelBias[2]); s.print(F(" ["));
          s.print(_model.state.accelBias[0]); s.print(' ');
          s.print(_model.state.accelBias[1]); s.print(' ');
          s.print(_model.state.accelBias[2]); s.println(F("]"));

          s.print(F(" mag offset: "));
          s.print(_model.config.magCalibrationOffset[0]); s.print(' ');
          s.print(_model.config.magCalibrationOffset[1]); s.print(' ');
          s.print(_model.config.magCalibrationOffset[2]); s.print(F(" ["));
          s.print(_model.state.magCalibrationOffset[0]); s.print(' ');
          s.print(_model.state.magCalibrationOffset[1]); s.print(' ');
          s.print(_model.state.magCalibrationOffset[2]); s.println(F("]"));

          s.print(F("  mag scale: "));
          s.print(_model.config.magCalibrationScale[0]); s.print(' ');
          s.print(_model.config.magCalibrationScale[1]); s.print(' ');
          s.print(_model.config.magCalibrationScale[2]); s.print(F(" ["));
          s.print(_model.state.magCalibrationScale[0]); s.print(' ');
          s.print(_model.state.magCalibrationScale[1]); s.print(' ');
          s.print(_model.state.magCalibrationScale[2]); s.println(F("]"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("gyro")) == 0)
        {
          if(!_model.isActive(MODE_ARMED)) _model.calibrateGyro();
          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("mag")) == 0)
        {
          if(!_model.isActive(MODE_ARMED)) _model.calibrateMag();
          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("reset_accel")) == 0 || strcmp_P(cmd.args[1], PSTR("reset_all")) == 0)
        {
          _model.state.accelBias = VectorFloat();
          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("reset_gyro")) == 0 || strcmp_P(cmd.args[1], PSTR("reset_all")) == 0)
        {
          _model.state.gyroBias = VectorFloat();
          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("reset_mag")) == 0 || strcmp_P(cmd.args[1], PSTR("reset_all")) == 0)
        {
          _model.state.magCalibrationOffset = VectorFloat();
          _model.state.magCalibrationScale = VectorFloat(1.f, 1.f, 1.f);
          s.println(F("OK"));
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("preset")) == 0)
      {
        if(!cmd.args[1])
        {
          s.println(F("Available presets: scaler, modes, micrus, brobot"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("scaler")) == 0)
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

          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("modes")) == 0)
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

          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("micrus")) == 0)
        {
          s.println(F("OK"));
        }
        else if(strcmp_P(cmd.args[1], PSTR("brobot")) == 0)
        {
          s.println(F("OK"));
        }
        else
        {
          s.println(F("NOT OK"));
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("load")) == 0)
      {
        _model.load();
        s.println(F("OK"));
      }
      else if(strcmp_P(cmd.args[0], PSTR("save")) == 0)
      {
        _model.save();
        s.println(F("OK"));
      }
      else if(strcmp_P(cmd.args[0], PSTR("eeprom")) == 0)
      {
        /*
        int start = 0;
        if(cmd.args[1])
        {
          start = std::max(String(cmd.args[1]).toInt(), 0L);
        }

        for(int i = start; i < start + 32; ++i)
        {
          uint8_t v = EEPROM.read(i);
          if(v <= 0xf) s.print('0');
          s.print(v, HEX);
          s.print(' ');
        }
        s.println();

        for(int i = start; i < start + 32; ++i)
        {
          s.print((int8_t)EEPROM.read(i));
          s.print(' ');
        }
        s.println();
        */
      }
      else if(strcmp_P(cmd.args[0], PSTR("scaler")) == 0)
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
          s.print(F("scaler: "));
          s.print(i);
          s.print(' ');
          s.print(mode);
          s.print(' ');
          s.print(min);
          s.print(' ');
          s.print(max);
          s.print(' ');
          s.print(v);
          s.print(' ');
          s.println(scale);
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("mixer")) == 0)
      {
        const MixerConfig& mixer = _model.state.currentMixer;
        s.print(F("set mix_outputs "));
        s.println(mixer.count);
        Param p;
        for(size_t i = 0; i < MIXER_RULE_MAX; i++)
        {
          s.print(F("set mix_"));
          s.print(i);
          s.print(' ');
          p.print(s, mixer.mixes[i]);
          s.println();
          if(mixer.mixes[i].src == MIXER_SOURCE_NULL) break;
        }
      }
      else if(strcmp_P(cmd.args[0], PSTR("status")) == 0)
      {

        printVersion(s);
        s.println();
        s.print(F("STATUS: "));
        s.println();
        s.print(F(" cpu : "));
        s.print(ESP.getCpuFreqMHz());
        s.println(F(" MHz"));

        Device::GyroDevice * gyro = Hardware::getGyroDevice(_model);
        Device::BaroDevice * baro = Hardware::getBaroDevice(_model);
        Device::MagDevice  * mag  = Hardware::getMagDevice(_model);
        if(gyro)
        {
          s.print(F("gyro : "));
          s.print(FPSTR(Device::GyroDevice::getName(gyro->getType())));
          s.print('/');
          s.println(FPSTR(Device::BusDevice::getName(gyro->getBus()->getType())));
        }
        else
        {
          s.println(F("gyro : NONE"));
        }

        if(baro)
        {
          s.print(F("baro : "));
          s.print(FPSTR(Device::BaroDevice::getName(baro->getType())));
          s.print('/');
          s.println(FPSTR(Device::BusDevice::getName(baro->getBus()->getType())));
        }
        else
        {
          s.println(F("baro : NONE"));
        }

        if(mag)
        {
          s.print(F(" mag : "));
          s.print(FPSTR(Device::MagDevice::getName(mag->getType())));
          s.print('/');
          s.println(FPSTR(Device::BusDevice::getName(mag->getBus()->getType())));
        }
        else
        {
          s.println(F(" mag : NONE"));
        }

        s.print(F(" rx rate : "));
        s.println(_model.state.inputFrameRate);

        s.print(F("arming disabled : "));
        s.println(_model.state.armingDisabledFlags);
      }
      else if(strcmp_P(cmd.args[0], PSTR("stats")) == 0)
      {
        printVersion(s);
        s.println();
        s.print(F("    cpu freq: "));
        s.print(ESP.getCpuFreqMHz());
        s.println(F(" MHz"));
        s.print(F("   gyro rate: "));
        s.print(_model.state.gyroTimer.rate);
        s.println(F(" Hz"));
        s.print(F("   loop rate: "));
        s.print(_model.state.loopTimer.rate);
        s.println(F(" Hz"));
        s.print(F("  mixer rate: "));
        s.print(_model.state.mixerTimer.rate);
        s.println(F(" Hz"));
        s.println();
        for(size_t i = 0; i < COUNTER_COUNT; ++i)
        {
          s.print(FPSTR(_model.state.stats.getName((StatCounter)i)));
          s.print(": ");
          s.print((int)(_model.state.stats.getTime((StatCounter)i) * _model.state.loopTimer.interval), 1);
          s.print("us, ");
          s.print(_model.state.stats.getLoad((StatCounter)i), 1);
          s.print("%");
          s.println();
        }
        s.print(F("  TOTAL: "));
        s.print((int)(_model.state.stats.getTotalTime() * _model.state.loopTimer.interval));
        s.print(F("us, "));
        s.print(_model.state.stats.getTotalLoad(), 1);
        s.print(F("%"));
        s.println();
      }
      else if(strcmp_P(cmd.args[0], PSTR("fsinfo")) == 0)
      {
        _model.logger.info(&s);
      }
      else if(strcmp_P(cmd.args[0], PSTR("fsformat")) == 0)
      {
        s.print(F("wait... "));
        _model.logger.format();
        s.println(F("OK"));
      }
      else if(strcmp_P(cmd.args[0], PSTR("logs")) == 0)
      {
        _model.logger.list(&s);
      }
      else if(strcmp_P(cmd.args[0], PSTR("log")) == 0)
      {
        if(!cmd.args[1])
        {
          _model.logger.show(&s);
          return;
        }
        int id = String(cmd.args[1]).toInt();
        if(!id)
        {
          s.println(F("invalid log id"));
          s.println();
          return;
        }
        _model.logger.show(&s, id);
      }
      else if(strcmp_P(cmd.args[0], PSTR("reboot")) == 0)
      {
        Hardware::restart(_model);
      }
      else if(strcmp_P(cmd.args[0], PSTR("defaults")) == 0)
      {
        _model.reset();
      }
      else if(strcmp_P(cmd.args[0], PSTR("exit")) == 0)
      {
        _active = false;
      }
      else
      {
        s.print(F("unknown command: "));
        s.println(cmd.args[0]);
      }
      s.println();
    }

  private:
    void print(const Param& param, Stream& s)
    {
      s.print(F("set "));
      s.print(FPSTR(param.name));
      s.print(' ');
      param.print(s);
      s.println();
    }

    void printVersion(Stream& s)
    {
      s.print(boardIdentifier);
      s.print(' ');
      s.print(targetName);
      s.print(' ');
      s.print(targetVersion);
      s.print(' ');
      s.print(shortGitRevision);
      s.print(' ');
      s.print(buildDate);
      s.print(' ');
      s.print(buildTime);
      s.print(' ');
      s.print(__VERSION__);
      s.print(' ');
      s.print(__cplusplus);
    }

    Model& _model;
    const Param * _params;
    bool _ignore;
    bool _active;
};

}

#endif
