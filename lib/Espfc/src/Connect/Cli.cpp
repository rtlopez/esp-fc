#include "Connect/Cli.hpp"
#include "Device/GyroDevice.hpp"
#include "Hardware.h"
#include "Utils/Filter.h"
#include "msp/msp_protocol.h"
#include <algorithm>
#include <cstring>
#include <iterator>
#include <platform.h>

#ifdef USE_FLASHFS
#include "Device/FlashDevice.h"
#endif

#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

#ifdef ESPFC_FREE_RTOS
#include <freertos/task.h>
#endif

namespace Espfc {

namespace Connect {

void Cli::Param::print(Stream& stream) const
{
  if (!addr)
  {
    stream.print("UNSET");
    return;
  }
  switch (type)
  {
    case PARAM_NONE:
      stream.print("NONE");
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
    case PARAM_BITMASK:
      stream.print((*reinterpret_cast<int32_t*>(addr) & (1ul << maxLen)) ? 1 : 0);
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

void Cli::Param::print(Stream& stream, const OutputChannelConfig& och) const
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

void Cli::Param::print(Stream& stream, const InputChannelConfig& ich) const
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

void Cli::Param::print(Stream& stream, const ScalerConfig& sc) const
{
  stream.print(sc.dimension);
  stream.print(' ');
  stream.print(sc.channel);
  stream.print(' ');
  stream.print(sc.minScale);
  stream.print(' ');
  stream.print(sc.maxScale);
}

void Cli::Param::print(Stream& stream, const ActuatorCondition& ac) const
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

void Cli::Param::print(Stream& stream, const MixerEntry& me) const
{
  stream.print(me.src);
  stream.print(' ');
  stream.print(me.dst);
  stream.print(' ');
  stream.print(me.rate);
}

void Cli::Param::print(Stream& stream, const SerialPortConfig& sc) const
{
  stream.print(sc.functionMask);
  stream.print(' ');
  stream.print(sc.baud);
  stream.print(' ');
  stream.print(sc.blackboxBaud);
}

void Cli::Param::print(Stream& stream, int32_t v) const
{
  if (choices)
  {
    for (int32_t i = 0; choices[i]; i++)
    {
      if (i == v)
      {
        stream.print(choices[i]);
        return;
      }
    }
  }
  stream.print(v);
}

void Cli::Param::update(const char** args) const
{
  const char* v = args[2];
  if (!addr) return;
  switch (type)
  {
    case PARAM_BOOL:
      if (!v) return;
      if (*v == '0') *addr = 0;
      if (*v == '1') *addr = 1;
      break;
    case PARAM_BYTE:
      if (!v) return;
      write((int8_t)parse(v));
      break;
    case PARAM_BYTE_U:
      if (!v) return;
      write((uint8_t)parse(v));
      break;
    case PARAM_SHORT:
      if (!v) return;
      write((int16_t)parse(v));
      break;
    case PARAM_INT:
      if (!v) return;
      write((int32_t)parse(v));
      break;
    case PARAM_FLOAT:
      if (!v) return;
      write(String(v).toFloat());
      break;
    case PARAM_STRING:
      write(String(v ? v : ""));
      break;
    case PARAM_BITMASK:
      if (!v) return;
      if (*v == '0')
      {
        *reinterpret_cast<int32_t*>(addr) &= ~(1ul << maxLen);
      }
      if (*v == '1')
      {
        *reinterpret_cast<int32_t*>(addr) |= (1ul << maxLen);
      }
      break;
    case PARAM_OUTPUT_CHANNEL:
      if (!v) return;
      write(*reinterpret_cast<OutputChannelConfig*>(addr), args);
      break;
    case PARAM_INPUT_CHANNEL:
      if (!v) return;
      write(*reinterpret_cast<InputChannelConfig*>(addr), args);
      break;
    case PARAM_SCALER:
      if (!v) return;
      write(*reinterpret_cast<ScalerConfig*>(addr), args);
      break;
    case PARAM_MODE:
      if (!v) return;
      write(*reinterpret_cast<ActuatorCondition*>(addr), args);
      break;
    case PARAM_MIXER:
      if (!v) return;
      write(*reinterpret_cast<MixerEntry*>(addr), args);
      break;
    case PARAM_SERIAL:
      if (!v) return;
      write(*reinterpret_cast<SerialPortConfig*>(addr), args);
      break;
    case PARAM_NONE:
      break;
  }
}

void Cli::Param::write(OutputChannelConfig& och, const char** args) const
{
  if (args[2]) och.servo = *args[2] == 'S';
  if (args[3]) och.reverse = *args[3] == 'R';
  if (args[4]) och.min = String(args[4]).toInt();
  if (args[5]) och.neutral = String(args[5]).toInt();
  if (args[6]) och.max = String(args[6]).toInt();
}

void Cli::Param::write(InputChannelConfig& ich, const char** args) const
{
  if (args[2]) ich.map = String(args[2]).toInt();
  if (args[3]) ich.min = String(args[3]).toInt();
  if (args[4]) ich.neutral = String(args[4]).toInt();
  if (args[5]) ich.max = String(args[5]).toInt();
  if (args[6]) ich.fsMode = *args[6] == 'A' ? 0 : (*args[6] == 'H' ? 1 : (*args[6] == 'S' ? 2 : 0));
  if (args[7]) ich.fsValue = String(args[7]).toInt();
}

void Cli::Param::write(ScalerConfig& sc, const char** args) const
{
  if (args[2]) sc.dimension = (ScalerDimension)String(args[2]).toInt();
  if (args[3]) sc.channel = String(args[3]).toInt();
  if (args[4]) sc.minScale = String(args[4]).toInt();
  if (args[5]) sc.maxScale = String(args[5]).toInt();
}

void Cli::Param::write(ActuatorCondition& ac, const char** args) const
{
  if (args[2]) ac.id = String(args[2]).toInt();
  if (args[3]) ac.ch = String(args[3]).toInt();
  if (args[4]) ac.min = String(args[4]).toInt();
  if (args[5]) ac.max = String(args[5]).toInt();
  if (args[6]) ac.logicMode = String(args[6]).toInt();
  if (args[7]) ac.linkId = String(args[7]).toInt();
}

void Cli::Param::write(MixerEntry& ac, const char** args) const
{
  if (args[2]) ac.src = constrain(String(args[2]).toInt(), 0, MIXER_SOURCE_MAX - 1);
  if (args[3]) ac.dst = constrain(String(args[3]).toInt(), 0, (int)(OUTPUT_CHANNELS - 1));
  if (args[4]) ac.rate = constrain(String(args[4]).toInt(), -1000, 1000);
}

void Cli::Param::write(SerialPortConfig& sc, const char** args) const
{
  if (args[2]) sc.functionMask = String(args[2]).toInt();
  if (args[3]) sc.baud = String(args[3]).toInt();
  if (args[4]) sc.blackboxBaud = String(args[4]).toInt();
}

void Cli::Param::write(const String& v) const
{
  *addr = 0;
  strncat(addr, v.c_str(), maxLen);
}

int32_t Cli::Param::parse(const char* v) const
{
  if (choices)
  {
    for (size_t i = 0; choices[i]; i++)
    {
      if (strcasecmp(v, choices[i]) == 0) return i;
    }
  }
  String tmp = v;
  return tmp.toInt();
}

Cli::Cli(Model& model): _model(model), _ignore(false), _active(false), _interactive(false)
{
  _params = initialize(_model.config);
}

const Cli::Param* Cli::initialize(ModelConfig& c)
{
  const char** busDevChoices = Device::BusDevice::getNames();
  const char** gyroDevChoices = Device::GyroDevice::getNames();
  const char** baroDevChoices = Device::BaroDevice::getNames();
  const char** magDevChoices = Device::MagDevice::getNames();

  const char** fusionModeChoices = FusionConfig::getModeNames();
  static const char* const* protocolChoices = EscDriver::getProtocolNames();

  // clang-format off
  static const char* gyroDlpfChoices[]   = { "256Hz", "188Hz", "98Hz", "42Hz", "20Hz", "10Hz", "5Hz", "EXPERIMENTAL", nullptr };
  static const char* debugModeChoices[]  = {  "NONE", "CYCLETIME", "BATTERY", "GYRO_FILTERED", "ACCELEROMETER", "PIDLOOP", "GYRO_SCALED", "RC_INTERPOLATION",
                                              "ANGLERATE", "ESC_SENSOR", "SCHEDULER", "STACK", "ESC_SENSOR_RPM", "ESC_SENSOR_TMP", "ALTITUDE", "FFT",
                                              "FFT_TIME", "FFT_FREQ", "RX_FRSKY_SPI", "RX_SFHSS_SPI", "GYRO_RAW", "DUAL_GYRO_RAW", "DUAL_GYRO_DIFF",
                                              "MAX7456_SIGNAL", "MAX7456_SPICLOCK", "SBUS", "FPORT", "RANGEFINDER", "RANGEFINDER_QUALITY", "LIDAR_TF",
                                              "ADC_INTERNAL", "RUNAWAY_TAKEOFF", "SDIO", "CURRENT_SENSOR", "USB", "SMARTAUDIO", "RTH", "ITERM_RELAX",
                                              "ACRO_TRAINER", "RC_SMOOTHING", "RX_SIGNAL_LOSS", "RC_SMOOTHING_RATE", "ANTI_GRAVITY", "DYN_LPF", "RX_SPEKTRUM_SPI",
                                              "DSHOT_RPM_TELEMETRY", "RPM_FILTER", "D_MIN", "AC_CORRECTION", "AC_ERROR", "DUAL_GYRO_SCALED", "DSHOT_RPM_ERRORS",
                                              "CRSF_LINK_STATISTICS_UPLINK", "CRSF_LINK_STATISTICS_PWR", "CRSF_LINK_STATISTICS_DOWN", "BARO", "GPS_RESCUE_THROTTLE_PID",
                                              "DYN_IDLE", "FF_LIMIT", "FF_INTERPOLATED", "BLACKBOX_OUTPUT", "GYRO_SAMPLE", "RX_TIMING", nullptr };
  static const char* filterTypeChoices[] = { "PT1", "BIQUAD", "PT2", "PT3", "NOTCH", "NOTCH_DF1", "BPF", "FO", "FIR2", "MEDIAN3", "NONE", nullptr };
  static const char* alignChoices[]      = { "DEFAULT", "CW0", "CW90", "CW180", "CW270", "CW0_FLIP", "CW90_FLIP", "CW180_FLIP", "CW270_FLIP", "CUSTOM", nullptr };
  static const char* mixerTypeChoices[]  = { "NONE", "TRI", "QUADP", "QUADX", "BI",
                                              "GIMBAL", "Y6", "HEX6", "FWING", "Y4",
                                              "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX", "AIRPLANE",
                                              "HELI120", "HELI90", "VTAIL4", "HEX6H", "PPMSERVO",
                                              "DUALCOPTER", "SINGLECOPTER", "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE",
                                              "CUSTOMTRI", "QUADX1234", nullptr };
  static const char* interpolChoices[]   = { "NONE", "DEFAULT", "AUTO", "MANUAL", nullptr };
  static const char* inputRateTypeChoices[] = { "BETAFLIGHT", "RACEFLIGHT", "KISS", "ACTUAL", "QUICK", nullptr };
  static const char* throtleLimitTypeChoices[] = { "NONE", "SCALE", "CLIP", nullptr };
  static const char* inputFilterChoices[] = { "INTERPOLATION", "FILTER", nullptr };
  static const char* inputItermRelaxChoices[] = { "OFF", "RP", "RPY", "RP_INC", "RPY_INC", nullptr };

  static const char* voltageSourceChoices[] = { "NONE", "ADC", nullptr };
  static const char* currentSourceChoices[] = { "NONE", "ADC", nullptr };
  static const char* blackboxDevChoices[] = { "NONE", "FLASH", "SD_CARD", "SERIAL", nullptr };
  static const char* blackboxModeChoices[] = { "NORMAL", "TEST", "ALWAYS", nullptr };
  static const char* ledTypeChoices[] = { "SIMPLE", "STRIP", nullptr };
  // clang-format on

  size_t i = 0;
  static const Param params[] = {

      Param("feature_gps", &c.featureMask, 7), Param("feature_dyn_notch", &c.featureMask, 29),
      Param("feature_motor_stop", &c.featureMask, 4), Param("feature_rx_ppm", &c.featureMask, 0),
      Param("feature_rx_serial", &c.featureMask, 3), Param("feature_rx_spi", &c.featureMask, 25),
      Param("feature_soft_serial", &c.featureMask, 6), Param("feature_telemetry", &c.featureMask, 10),

      Param("debug_mode", &c.debug.mode, debugModeChoices), Param("debug_axis", &c.debug.axis),

      Param("gyro_bus", &c.gyro.bus, busDevChoices), Param("gyro_dev", &c.gyro.dev, gyroDevChoices),
      Param("gyro_dlpf", &c.gyro.dlpf, gyroDlpfChoices), Param("gyro_align", &c.gyro.align, alignChoices),
      Param("gyro_lpf_type", &c.gyro.filter.type, filterTypeChoices), Param("gyro_lpf_freq", &c.gyro.filter.freq),
      Param("gyro_lpf2_type", &c.gyro.filter2.type, filterTypeChoices), Param("gyro_lpf2_freq", &c.gyro.filter2.freq),
      Param("gyro_lpf3_type", &c.gyro.filter3.type, filterTypeChoices), Param("gyro_lpf3_freq", &c.gyro.filter3.freq),
      Param("gyro_notch1_freq", &c.gyro.notch1Filter.freq), Param("gyro_notch1_cutoff", &c.gyro.notch1Filter.cutoff),
      Param("gyro_notch2_freq", &c.gyro.notch2Filter.freq), Param("gyro_notch2_cutoff", &c.gyro.notch2Filter.cutoff),
      Param("gyro_dyn_lpf_min", &c.gyro.dynLpfFilter.cutoff), Param("gyro_dyn_lpf_max", &c.gyro.dynLpfFilter.freq),
      Param("gyro_dyn_notch_q", &c.gyro.dynamicFilter.q), Param("gyro_dyn_notch_count", &c.gyro.dynamicFilter.count),
      Param("gyro_dyn_notch_min", &c.gyro.dynamicFilter.min_freq),
      Param("gyro_dyn_notch_max", &c.gyro.dynamicFilter.max_freq),
      Param("gyro_rpm_harmonics", &c.gyro.rpmFilter.harmonics), Param("gyro_rpm_q", &c.gyro.rpmFilter.q),
      Param("gyro_rpm_min_freq", &c.gyro.rpmFilter.minFreq), Param("gyro_rpm_fade", &c.gyro.rpmFilter.fade),
      Param("gyro_rpm_weight_1", &c.gyro.rpmFilter.weights[0]),
      Param("gyro_rpm_weight_2", &c.gyro.rpmFilter.weights[1]),
      Param("gyro_rpm_weight_3", &c.gyro.rpmFilter.weights[2]),
      Param("gyro_rpm_tlm_lpf_freq", &c.gyro.rpmFilter.freqLpf), Param("gyro_offset_x", &c.gyro.bias[0]),
      Param("gyro_offset_y", &c.gyro.bias[1]), Param("gyro_offset_z", &c.gyro.bias[2]),

      Param("accel_bus", &c.accel.bus, busDevChoices), Param("accel_dev", &c.accel.dev, gyroDevChoices),
      Param("accel_lpf_type", &c.accel.filter.type, filterTypeChoices), Param("accel_lpf_freq", &c.accel.filter.freq),
      Param("accel_offset_x", &c.accel.bias[0]), Param("accel_offset_y", &c.accel.bias[1]),
      Param("accel_offset_z", &c.accel.bias[2]), Param("accel_trim_roll", &c.accel.trim[1]),
      Param("accel_trim_pitch", &c.accel.trim[0]),

      Param("mag_bus", &c.mag.bus, busDevChoices), Param("mag_dev", &c.mag.dev, magDevChoices),
      Param("mag_align", &c.mag.align, alignChoices), Param("mag_filter_type", &c.mag.filter.type, filterTypeChoices),
      Param("mag_filter_lpf", &c.mag.filter.freq), Param("mag_offset_x", &c.mag.offset[0]),
      Param("mag_offset_y", &c.mag.offset[1]), Param("mag_offset_z", &c.mag.offset[2]),
      Param("mag_scale_x", &c.mag.scale[0]), Param("mag_scale_y", &c.mag.scale[1]),
      Param("mag_scale_z", &c.mag.scale[2]),

      Param("baro_bus", &c.baro.bus, busDevChoices), Param("baro_dev", &c.baro.dev, baroDevChoices),
      Param("baro_lpf_type", &c.baro.filter.type, filterTypeChoices), Param("baro_lpf_freq", &c.baro.filter.freq),

      Param("gps_min_sats", &c.gps.minSats), Param("gps_set_home_once", &c.gps.setHomeOnce),

      Param("gps_gnss_mode", &c.gps.gnssMode), Param("gps_enable_dual_band", &c.gps.enableDualBand),
      Param("gps_enable_gps", &c.gps.enableGPS), Param("gps_enable_glonass", &c.gps.enableGLONASS),
      Param("gps_enable_galileo", &c.gps.enableGalileo), Param("gps_enable_beidou", &c.gps.enableBeiDou),
      Param("gps_enable_qzss", &c.gps.enableQZSS), Param("gps_enable_sbas", &c.gps.enableSBAS),

      Param("board_align_roll", &c.boardAlignment[0]), Param("board_align_pitch", &c.boardAlignment[1]),
      Param("board_align_yaw", &c.boardAlignment[2]),

      Param("vbat_source", &c.vbat.source, voltageSourceChoices), Param("vbat_scale", &c.vbat.scale),
      Param("vbat_mul", &c.vbat.resMult), Param("vbat_div", &c.vbat.resDiv),
      Param("vbat_cell_warn", &c.vbat.cellWarning),

      Param("ibat_source", &c.ibat.source, currentSourceChoices), Param("ibat_scale", &c.ibat.scale),
      Param("ibat_offset", &c.ibat.offset),

      Param("fusion_mode", &c.fusion.mode, fusionModeChoices), Param("fusion_gain_p", &c.fusion.gain),
      Param("fusion_gain_i", &c.fusion.gainI), Param("fusion_use_mag", &c.fusion.useMag),

      Param("input_rate_type", &c.input.rateType, inputRateTypeChoices),

      Param("input_roll_rate", &c.input.rate[0]), Param("input_roll_srate", &c.input.superRate[0]),
      Param("input_roll_expo", &c.input.expo[0]), Param("input_roll_limit", &c.input.rateLimit[0]),

      Param("input_pitch_rate", &c.input.rate[1]), Param("input_pitch_srate", &c.input.superRate[1]),
      Param("input_pitch_expo", &c.input.expo[1]), Param("input_pitch_limit", &c.input.rateLimit[1]),

      Param("input_yaw_rate", &c.input.rate[2]), Param("input_yaw_srate", &c.input.superRate[2]),
      Param("input_yaw_expo", &c.input.expo[2]), Param("input_yaw_limit", &c.input.rateLimit[2]),

      Param("input_deadband", &c.input.deadband),

      Param("input_min", &c.input.minRc), Param("input_mid", &c.input.midRc), Param("input_max", &c.input.maxRc),

      Param("input_interpolation", &c.input.interpolationMode, interpolChoices),
      Param("input_interpolation_interval", &c.input.interpolationInterval),

      Param("input_filter_type", &c.input.filterType, inputFilterChoices),
      Param("input_lpf_type", &c.input.filter.type, filterTypeChoices), Param("input_lpf_freq", &c.input.filter.freq),
      Param("input_lpf_factor", &c.input.filterAutoFactor),
      Param("input_ff_lpf_type", &c.input.filterDerivative.type, filterTypeChoices),
      Param("input_ff_lpf_freq", &c.input.filterDerivative.freq),

      Param("input_rssi_channel", &c.input.rssiChannel),

      Param("input_0", &c.input.channel[0]), Param("input_1", &c.input.channel[1]),
      Param("input_2", &c.input.channel[2]), Param("input_3", &c.input.channel[3]),
      Param("input_4", &c.input.channel[4]), Param("input_5", &c.input.channel[5]),
      Param("input_6", &c.input.channel[6]), Param("input_7", &c.input.channel[7]),
      Param("input_8", &c.input.channel[8]), Param("input_9", &c.input.channel[9]),
      Param("input_10", &c.input.channel[10]), Param("input_11", &c.input.channel[11]),
      Param("input_12", &c.input.channel[12]), Param("input_13", &c.input.channel[13]),
      Param("input_14", &c.input.channel[14]), Param("input_15", &c.input.channel[15]),

      Param("failsafe_delay", &c.failsafe.delay), Param("failsafe_kill_switch", &c.failsafe.killSwitch),

      Param("arming_small_angle", &c.arming.smallAngle),

      Param("vtx_power", &c.vtx.power), Param("vtx_channel", &c.vtx.channel), Param("vtx_band", &c.vtx.band),
      Param("vtx_low_power_disarm", &c.vtx.lowPowerDisarm),

#ifdef ESPFC_SERIAL_0
      Param("serial_0", &c.serial[SERIAL_UART_0]),
#endif
#ifdef ESPFC_SERIAL_1
      Param("serial_1", &c.serial[SERIAL_UART_1]),
#endif
#ifdef ESPFC_SERIAL_2
      Param("serial_2", &c.serial[SERIAL_UART_2]),
#endif
#ifdef ESPFC_SERIAL_SOFT_0
      Param("serial_soft_0", &c.serial[SERIAL_SOFT_0]),
#endif
#ifdef ESPFC_SERIAL_USB
      Param("serial_usb", &c.serial[SERIAL_USB]),
#endif

      Param("scaler_0", &c.scaler[0]), Param("scaler_1", &c.scaler[1]), Param("scaler_2", &c.scaler[2]),

      Param("mode_0", &c.conditions[0]), Param("mode_1", &c.conditions[1]), Param("mode_2", &c.conditions[2]),
      Param("mode_3", &c.conditions[3]), Param("mode_4", &c.conditions[4]), Param("mode_5", &c.conditions[5]),
      Param("mode_6", &c.conditions[6]), Param("mode_7", &c.conditions[7]),

      Param("pid_sync", &c.loopSync),

      Param("pid_roll_p", &c.pid[FC_PID_ROLL].P), Param("pid_roll_i", &c.pid[FC_PID_ROLL].I),
      Param("pid_roll_d", &c.pid[FC_PID_ROLL].D), Param("pid_roll_f", &c.pid[FC_PID_ROLL].F),

      Param("pid_pitch_p", &c.pid[FC_PID_PITCH].P), Param("pid_pitch_i", &c.pid[FC_PID_PITCH].I),
      Param("pid_pitch_d", &c.pid[FC_PID_PITCH].D), Param("pid_pitch_f", &c.pid[FC_PID_PITCH].F),

      Param("pid_yaw_p", &c.pid[FC_PID_YAW].P), Param("pid_yaw_i", &c.pid[FC_PID_YAW].I),
      Param("pid_yaw_d", &c.pid[FC_PID_YAW].D), Param("pid_yaw_f", &c.pid[FC_PID_YAW].F),

      Param("pid_level_p", &c.pid[FC_PID_LEVEL].P), Param("pid_level_i", &c.pid[FC_PID_LEVEL].I),
      Param("pid_level_d", &c.pid[FC_PID_LEVEL].D), Param("pid_level_f", &c.pid[FC_PID_LEVEL].F),

      Param("pid_level_angle_limit", &c.level.angleLimit), Param("pid_level_rate_limit", &c.level.rateLimit),
      Param("pid_level_lpf_type", &c.level.ptermFilter.type, filterTypeChoices),
      Param("pid_level_lpf_freq", &c.level.ptermFilter.freq),

      Param("pid_althold_vel_p", &c.pid[FC_PID_VEL].P), Param("pid_althold_vel_i", &c.pid[FC_PID_VEL].I),
      Param("pid_althold_vel_d", &c.pid[FC_PID_VEL].D), Param("pid_althold_vel_f", &c.pid[FC_PID_VEL].F),
      Param("pid_althold_iterm_center", &c.altHold.itermCenter),
      Param("pid_althold_iterm_range", &c.altHold.itermRange), Param("pid_althold_baro_tau", &c.altHold.baroTau),

      Param("pid_yaw_lpf_type", &c.yaw.filter.type, filterTypeChoices), Param("pid_yaw_lpf_freq", &c.yaw.filter.freq),

      Param("pid_dterm_lpf_type", &c.dterm.filter.type, filterTypeChoices),
      Param("pid_dterm_lpf_freq", &c.dterm.filter.freq),
      Param("pid_dterm_lpf2_type", &c.dterm.filter2.type, filterTypeChoices),
      Param("pid_dterm_lpf2_freq", &c.dterm.filter2.freq), Param("pid_dterm_notch_freq", &c.dterm.notchFilter.freq),
      Param("pid_dterm_notch_cutoff", &c.dterm.notchFilter.cutoff),
      Param("pid_dterm_dyn_lpf_min", &c.dterm.dynLpfFilter.cutoff),
      Param("pid_dterm_dyn_lpf_max", &c.dterm.dynLpfFilter.freq),

      Param("pid_dterm_weight", &c.dterm.setpointWeight), Param("pid_iterm_limit", &c.iterm.limit),
      Param("pid_iterm_zero", &c.iterm.lowThrottleZeroIterm),
      Param("pid_iterm_relax", &c.iterm.relax, inputItermRelaxChoices),
      Param("pid_iterm_relax_cutoff", &c.iterm.relaxCutoff), Param("pid_tpa_scale", &c.controller.tpaScale),
      Param("pid_tpa_breakpoint", &c.controller.tpaBreakpoint),

      Param("mixer_sync", &c.mixerSync), Param("mixer_type", &c.mixer.type, mixerTypeChoices),
      Param("mixer_yaw_reverse", &c.mixer.yawReverse),
      Param("mixer_throttle_limit_type", &c.output.throttleLimitType, throtleLimitTypeChoices),
      Param("mixer_throttle_limit_percent", &c.output.throttleLimitPercent),
      Param("mixer_output_limit", &c.output.motorLimit),

      Param("output_motor_protocol", &c.output.protocol, protocolChoices), Param("output_motor_async", &c.output.async),
      Param("output_motor_rate", &c.output.rate),
      Param("output_motor_idle", &c.output.motorIdle),
#ifdef ESPFC_DSHOT_TELEMETRY
      Param("output_motor_poles", &c.output.motorPoles),
      Param("output_dshot_telemetry", &c.output.dshotTelemetry),
#endif
      Param("output_servo_rate", &c.output.servoRate),

      Param("output_min_command", &c.output.minCommand),
      Param("output_max_throttle", &c.output.maxThrottle),
      Param("output_0", &c.output.channel[0]), Param("output_1", &c.output.channel[1]),
      Param("output_2", &c.output.channel[2]), Param("output_3", &c.output.channel[3]),
#if ESPFC_OUTPUT_COUNT > 4
      Param("output_4", &c.output.channel[4]),
#endif
#if ESPFC_OUTPUT_COUNT > 5
      Param("output_5", &c.output.channel[5]),
#endif
#if ESPFC_OUTPUT_COUNT > 6
      Param("output_6", &c.output.channel[6]),
#endif
#if ESPFC_OUTPUT_COUNT > 7
      Param("output_7", &c.output.channel[7]),
#endif
#ifdef ESPFC_INPUT
      Param("pin_input_rx", &c.pin[PIN_INPUT_RX]),
#endif
      Param("pin_output_0", &c.pin[PIN_OUTPUT_0]), Param("pin_output_1", &c.pin[PIN_OUTPUT_1]),
      Param("pin_output_2", &c.pin[PIN_OUTPUT_2]), Param("pin_output_3", &c.pin[PIN_OUTPUT_3]),
#if ESPFC_OUTPUT_COUNT > 4
      Param("pin_output_4", &c.pin[PIN_OUTPUT_4]),
#endif
#if ESPFC_OUTPUT_COUNT > 5
      Param("pin_output_5", &c.pin[PIN_OUTPUT_5]),
#endif
#if ESPFC_OUTPUT_COUNT > 6
      Param("pin_output_6", &c.pin[PIN_OUTPUT_6]),
#endif
#if ESPFC_OUTPUT_COUNT > 7
      Param("pin_output_7", &c.pin[PIN_OUTPUT_7]),
#endif
      Param("pin_button", &c.pin[PIN_BUTTON]), Param("pin_buzzer", &c.pin[PIN_BUZZER]),
      Param("pin_led", &c.pin[PIN_LED_BLINK]),
#if defined(ESPFC_SERIAL_0) && defined(ESPFC_SERIAL_REMAP_PINS)
      Param("pin_serial_0_tx", &c.pin[PIN_SERIAL_0_TX]), Param("pin_serial_0_rx", &c.pin[PIN_SERIAL_0_RX]),
#endif
#if defined(ESPFC_SERIAL_1) && defined(ESPFC_SERIAL_REMAP_PINS)
      Param("pin_serial_1_tx", &c.pin[PIN_SERIAL_1_TX]), Param("pin_serial_1_rx", &c.pin[PIN_SERIAL_1_RX]),
#endif
#if defined(ESPFC_SERIAL_2) && defined(ESPFC_SERIAL_REMAP_PINS)
      Param("pin_serial_2_tx", &c.pin[PIN_SERIAL_2_TX]), Param("pin_serial_2_rx", &c.pin[PIN_SERIAL_2_RX]),
#endif
#ifdef ESPFC_I2C_0
      Param("pin_i2c_scl", &c.pin[PIN_I2C_0_SCL]), Param("pin_i2c_sda", &c.pin[PIN_I2C_0_SDA]),
#endif
#ifdef ESPFC_ADC_0
      Param("pin_input_adc_0", &c.pin[PIN_INPUT_ADC_0]),
#endif
#ifdef ESPFC_ADC_1
      Param("pin_input_adc_1", &c.pin[PIN_INPUT_ADC_1]),
#endif
#ifdef ESPFC_SPI_0
      Param("pin_spi_0_sck", &c.pin[PIN_SPI_0_SCK]), Param("pin_spi_0_mosi", &c.pin[PIN_SPI_0_MOSI]),
      Param("pin_spi_0_miso", &c.pin[PIN_SPI_0_MISO]), Param("pin_spi_cs_0", &c.pin[PIN_SPI_CS0]),
      Param("pin_spi_cs_1", &c.pin[PIN_SPI_CS1]), Param("pin_spi_cs_2", &c.pin[PIN_SPI_CS2]),
#endif
      Param("pin_buzzer_invert", &c.buzzer.inverted), Param("pin_led_invert", &c.led.invert),
      Param("pin_led_type", &c.led.type, ledTypeChoices),

#ifdef ESPFC_I2C_0
      Param("i2c_speed", &c.i2cSpeed),
#endif
      Param("rescue_config_delay", &c.rescueConfigDelay),

      // Param("telemetry", &c.telemetry),
      Param("telemetry_interval", &c.telemetryInterval),

      Param("blackbox_dev", &c.blackbox.dev, blackboxDevChoices),
      Param("blackbox_mode", &c.blackbox.mode, blackboxModeChoices), Param("blackbox_rate", &c.blackbox.pDenom),
      Param("blackbox_log_acc", &c.blackbox.fieldsMask, BLACKBOX_FIELD_ACC),
      Param("blackbox_log_alt", &c.blackbox.fieldsMask, BLACKBOX_FIELD_ALTITUDE),
      Param("blackbox_log_bat", &c.blackbox.fieldsMask, BLACKBOX_FIELD_BATTERY),
      Param("blackbox_log_debug", &c.blackbox.fieldsMask, BLACKBOX_FIELD_DEBUG_LOG),
      Param("blackbox_log_gps", &c.blackbox.fieldsMask, BLACKBOX_FIELD_GPS),
      Param("blackbox_log_gyro", &c.blackbox.fieldsMask, BLACKBOX_FIELD_GYRO),
      Param("blackbox_log_gyro_raw", &c.blackbox.fieldsMask, BLACKBOX_FIELD_GYROUNFILT),
      Param("blackbox_log_mag", &c.blackbox.fieldsMask, BLACKBOX_FIELD_MAG),
      Param("blackbox_log_motor", &c.blackbox.fieldsMask, BLACKBOX_FIELD_MOTOR),
      Param("blackbox_log_pid", &c.blackbox.fieldsMask, BLACKBOX_FIELD_PID),
      Param("blackbox_log_rc", &c.blackbox.fieldsMask, BLACKBOX_FIELD_RC_COMMANDS),
      Param("blackbox_log_rpm", &c.blackbox.fieldsMask, BLACKBOX_FIELD_RPM),
      Param("blackbox_log_rssi", &c.blackbox.fieldsMask, BLACKBOX_FIELD_RSSI),
      Param("blackbox_log_sp", &c.blackbox.fieldsMask, BLACKBOX_FIELD_SETPOINT),

      Param("model_name", PARAM_STRING, &c.modelName[0], nullptr, MODEL_NAME_LEN),

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
      Param("wifi_ssid", PARAM_STRING, &c.wireless.ssid[0], nullptr, WirelessConfig::MAX_LEN),
      Param("wifi_pass", PARAM_STRING, &c.wireless.pass[0], nullptr, WirelessConfig::MAX_LEN),
      Param("wifi_tcp_port", &c.wireless.port),
#endif

      Param("mix_outputs", &c.customMixerCount), Param("mix_0", &c.customMixes[i++]),
      Param("mix_1", &c.customMixes[i++]), Param("mix_2", &c.customMixes[i++]), Param("mix_3", &c.customMixes[i++]),
      Param("mix_4", &c.customMixes[i++]), Param("mix_5", &c.customMixes[i++]), Param("mix_6", &c.customMixes[i++]),
      Param("mix_7", &c.customMixes[i++]), Param("mix_8", &c.customMixes[i++]), Param("mix_9", &c.customMixes[i++]),
      Param("mix_10", &c.customMixes[i++]), Param("mix_11", &c.customMixes[i++]), Param("mix_12", &c.customMixes[i++]),
      Param("mix_13", &c.customMixes[i++]), Param("mix_14", &c.customMixes[i++]), Param("mix_15", &c.customMixes[i++]),
      Param("mix_16", &c.customMixes[i++]), Param("mix_17", &c.customMixes[i++]), Param("mix_18", &c.customMixes[i++]),
      Param("mix_19", &c.customMixes[i++]), Param("mix_20", &c.customMixes[i++]), Param("mix_21", &c.customMixes[i++]),
      Param("mix_22", &c.customMixes[i++]), Param("mix_23", &c.customMixes[i++]), Param("mix_24", &c.customMixes[i++]),
      Param("mix_25", &c.customMixes[i++]), Param("mix_26", &c.customMixes[i++]), Param("mix_27", &c.customMixes[i++]),
      Param("mix_28", &c.customMixes[i++]), Param("mix_29", &c.customMixes[i++]), Param("mix_30", &c.customMixes[i++]),
      Param("mix_31", &c.customMixes[i++]), Param("mix_32", &c.customMixes[i++]), Param("mix_33", &c.customMixes[i++]),
      Param("mix_34", &c.customMixes[i++]), Param("mix_35", &c.customMixes[i++]), Param("mix_36", &c.customMixes[i++]),
      Param("mix_37", &c.customMixes[i++]), Param("mix_38", &c.customMixes[i++]), Param("mix_39", &c.customMixes[i++]),
      Param("mix_40", &c.customMixes[i++]), Param("mix_41", &c.customMixes[i++]), Param("mix_42", &c.customMixes[i++]),
      Param("mix_43", &c.customMixes[i++]), Param("mix_44", &c.customMixes[i++]), Param("mix_45", &c.customMixes[i++]),
      Param("mix_46", &c.customMixes[i++]), Param("mix_47", &c.customMixes[i++]), Param("mix_48", &c.customMixes[i++]),
      Param("mix_49", &c.customMixes[i++]), Param("mix_50", &c.customMixes[i++]), Param("mix_51", &c.customMixes[i++]),
      Param("mix_52", &c.customMixes[i++]), Param("mix_53", &c.customMixes[i++]), Param("mix_54", &c.customMixes[i++]),
      Param("mix_55", &c.customMixes[i++]), Param("mix_56", &c.customMixes[i++]), Param("mix_57", &c.customMixes[i++]),
      Param("mix_58", &c.customMixes[i++]), Param("mix_59", &c.customMixes[i++]), Param("mix_60", &c.customMixes[i++]),
      Param("mix_61", &c.customMixes[i++]), Param("mix_62", &c.customMixes[i++]), Param("mix_63", &c.customMixes[i++]),

      Param() // terminate
  };
  return params;
}

bool Cli::process(const char c, CliCmd& cmd, Stream& stream)
{
  // configurator handshake
  if (!_active && c == '#')
  {
    // FIXME: detect disconnection
    _active = true;
    _interactive = true;
    stream.println();
    stream.println("Entering CLI Mode, type 'exit' to return, or 'help'");
    stream.print("# ");
    printVersion(stream);
    stream.println();
    _model.setArmingDisabled(ARMING_DISABLED_CLI, true);
    cmd = CliCmd();
    return true;
  }

  // non-interactive session enter byte 0x02
  if (!_active && !_interactive && c == 2)
  {
    _active = true;
    cmd = CliCmd();
    stream.write(2);
    return true;
  }
  // non-interactive session exit byte 0x03
  if (_active && !_interactive && c == 3)
  {
    _active = false;
    cmd = CliCmd();
    stream.write(3);
    return true;
  }

  // CTRL-D
  if (_active && c == 4)
  {
    stream.println();
    stream.println(" #leaving CLI mode, unsaved changes lost");
    _active = false;
    _interactive = false;
    cmd = CliCmd();
    return true;
  }

  // execute on end line
  bool endl = c == '\n' || c == '\r';
  if (cmd.index && endl)
  {
    parse(cmd);
    execute(cmd, stream);
    cmd = CliCmd();
    return true;
  }

  // ignore comments
  if (c == '#')
  {
    _ignore = true;
  }
  else if (endl)
  {
    _ignore = false;
  }

  // don't put characters into buffer in specific conditions
  if (_ignore || endl || cmd.index >= CLI_BUFF_SIZE - 1)
  {
    return false;
  }

  if (c == '\b') // handle backspace
  {
    if (cmd.index)
    {
      cmd.buff[--cmd.index] = '\0';
    }
  }
  else
  {
    if (!_active)
    {
      _active = true;
      _interactive = true;
    }
    cmd.buff[cmd.index] = c;
    cmd.buff[++cmd.index] = '\0';
  }
  return false;
}

void Cli::parse(CliCmd& cmd)
{
  const char* DELIM = " \t";
  char* pch = strtok(cmd.buff, DELIM);
  size_t count = 0;
  while (pch)
  {
    cmd.args[count++] = pch;
    pch = strtok(nullptr, DELIM);
  }
}

void Cli::execute(CliCmd& cmd, Stream& s)
{
  if (_interactive)
  {
    if (cmd.args[0]) s.print("# ");
    for (size_t i = 0; i < CLI_ARGS_SIZE; ++i)
    {
      if (!cmd.args[i]) break;
      s.print(cmd.args[i]);
      s.print(' ');
    }
    s.println();
  }

  if (!cmd.args[0]) return;

  if (strcmp(cmd.args[0], "help") == 0)
  {
    static const char* const helps[] = {"available commands:", " help", " dump", " get param", " set param value ...",
                                        " cal [gyro]", " defaults", " save", " reboot", " scaler", " mixer", " stats",
                                        " status", " devinfo", " version", " logs", " gps [set_home|clear_home]",
                                        //" load", " eeprom",
                                        //" fsinfo", " fsformat", " log",
                                        nullptr};
    for (const char* const* ptr = helps; *ptr; ptr++)
    {
      s.println(*ptr);
    }
  }
  else if (strcmp(cmd.args[0], "version") == 0)
  {
    printVersion(s);
    s.println();
  }
#if defined(ESPFC_WIFI) || defined(ESPFC_WIFI_ALT)
  else if (strcmp(cmd.args[0], "wifi") == 0)
  {
    s.print("ST IP4: tcp://");
    s.print(WiFi.localIP());
    s.print(":");
    s.println(_model.config.wireless.port);
    s.print("ST MAC: ");
    s.println(WiFi.macAddress());
    s.print("AP IP4: tcp://");
    s.print(WiFi.softAPIP());
    s.print(":");
    s.println(_model.config.wireless.port);
    s.print("AP MAC: ");
    s.println(WiFi.softAPmacAddress());
    s.print("STATUS: ");
    s.println(WiFi.status());
    s.print("  MODE: ");
    s.println(WiFi.getMode());
    s.print("CHANNEL: ");
    s.println(WiFi.channel());
    // WiFi.printDiag(s);
  }
#endif
#if defined(ESPFC_FREE_RTOS)
  else if (strcmp(cmd.args[0], "tasks") == 0)
  {
    printVersion(s);
    s.println();

    size_t numTasks = uxTaskGetNumberOfTasks();

    s.print("num tasks: ");
    s.print(numTasks);
    s.println();
  }
#endif
  else if (strcmp(cmd.args[0], "devinfo") == 0)
  {
    printVersion(s);
    s.println();

    s.print("cpu freq: ");
    s.print(targetCpuFreq());
    s.println(" MHz");

    s.print("  memory: ");
    s.print(sizeof(ModelConfig));
    s.print(", ");
    s.print(sizeof(ModelState));
    s.print(", ");
    s.println(targetFreeHeap());
  }
  else if (strcmp(cmd.args[0], "get") == 0)
  {
    if (strcmp(cmd.args[1], "mag_calibration") == 0)
    {
      // BF specific required by configurator
      s.print("mag_calibration = ");
      s.print(lrintf(_model.state.mag.calibrationOffset[0] * 10.f));
      s.print(",");
      s.print(lrintf(_model.state.mag.calibrationOffset[1] * 10.f));
      s.print(",");
      s.print(lrintf(_model.state.mag.calibrationOffset[2] * 10.f));
      s.println();
      return;
    }
    bool found = false;
    for (size_t i = 0; _params[i].name; ++i)
    {
      String ts = _params[i].name;
      if (!cmd.args[1] || ts.indexOf(cmd.args[1]) >= 0)
      {
        print(_params[i], s);
        found = true;
      }
    }
    if (!found)
    {
      s.print("param not found: ");
      s.print(cmd.args[1]);
    }
    s.println();
  }
  else if (strcmp(cmd.args[0], "set") == 0)
  {
    if (!cmd.args[1])
    {
      s.println("param required");
      s.println();
      return;
    }
    bool found = false;
    for (size_t i = 0; _params[i].name; ++i)
    {
      if (strcmp(cmd.args[1], _params[i].name) == 0)
      {
        _params[i].update(cmd.args);
        print(_params[i], s);
        found = true;
        break;
      }
    }
    if (!found)
    {
      s.print("param not found: ");
      s.println(cmd.args[1]);
    }
  }
  else if (strcmp(cmd.args[0], "dump") == 0)
  {
    s.println("defaults");
    for (size_t i = 0; _params[i].name; ++i)
    {
      print(_params[i], s);
    }
    s.println("save");
  }
  else if (strcmp(cmd.args[0], "sensor_hardware") == 0)
  {
    // BF specific required by configurator
    s.print("gyro: ");
    const auto* gyroAccNames = Device::GyroDevice::getNames();
    for(size_t i = 0; gyroAccNames[i]; ++i)
    {
      if (i) s.print(',');
      s.print(gyroAccNames[i]);
    }
    s.println();

    s.print("acc: ");
    for (size_t i = 0; gyroAccNames[i]; i++)
    {
      if (i) s.print(',');
      s.print(gyroAccNames[i]);
    }
    s.println();

    s.print("baro: ");
    const auto* baroNames = Device::BaroDevice::getNames();
    for(size_t i = 0; baroNames[i]; ++i)
    {
      if (i) s.print(',');
      s.print(baroNames[i]);
    }
    s.println();

    s.print("mag: ");
    const auto* magNames = Device::MagDevice::getNames();
    for(size_t i = 0; magNames[i]; ++i)
    {
      if (i) s.print(',');
      s.print(magNames[i]);
    }
    s.println();

    s.println("rangefinder: NONE");
    s.println("opticalflow: NONE");
  }
  else if (strcmp(cmd.args[0], "cal") == 0)
  {
    if (!cmd.args[1])
    {
      s.print(" gyro offset: ");
      s.print(_model.config.gyro.bias[0]);
      s.print(' ');
      s.print(_model.config.gyro.bias[1]);
      s.print(' ');
      s.print(_model.config.gyro.bias[2]);
      s.print(" [");
      s.print(Utils::toDeg(_model.state.gyro.bias[0]));
      s.print(' ');
      s.print(Utils::toDeg(_model.state.gyro.bias[1]));
      s.print(' ');
      s.print(Utils::toDeg(_model.state.gyro.bias[2]));
      s.println("]");

      s.print("accel offset: ");
      s.print(_model.config.accel.bias[0]);
      s.print(' ');
      s.print(_model.config.accel.bias[1]);
      s.print(' ');
      s.print(_model.config.accel.bias[2]);
      s.print(" [");
      s.print(_model.state.accel.bias[0]);
      s.print(' ');
      s.print(_model.state.accel.bias[1]);
      s.print(' ');
      s.print(_model.state.accel.bias[2]);
      s.println("]");

      s.print("  mag offset: ");
      s.print(_model.config.mag.offset[0]);
      s.print(' ');
      s.print(_model.config.mag.offset[1]);
      s.print(' ');
      s.print(_model.config.mag.offset[2]);
      s.print(" [");
      s.print(_model.state.mag.calibrationOffset[0]);
      s.print(' ');
      s.print(_model.state.mag.calibrationOffset[1]);
      s.print(' ');
      s.print(_model.state.mag.calibrationOffset[2]);
      s.println("]");

      s.print("   mag scale: ");
      s.print(_model.config.mag.scale[0]);
      s.print(' ');
      s.print(_model.config.mag.scale[1]);
      s.print(' ');
      s.print(_model.config.mag.scale[2]);
      s.print(" [");
      s.print(_model.state.mag.calibrationScale[0]);
      s.print(' ');
      s.print(_model.state.mag.calibrationScale[1]);
      s.print(' ');
      s.print(_model.state.mag.calibrationScale[2]);
      s.println("]");
    }
    else if (strcmp(cmd.args[1], "gyro") == 0)
    {
      if (!_model.isModeActive(MODE_ARMED)) _model.calibrateGyro();
      s.println("OK");
    }
    else if (strcmp(cmd.args[1], "mag") == 0)
    {
      if (!_model.isModeActive(MODE_ARMED)) _model.calibrateMag();
      s.println("OK");
    }
    else
    {
      if (strcmp(cmd.args[1], "reset_accel") == 0 || strcmp(cmd.args[1], "reset_all") == 0)
      {
        _model.state.accel.bias = {};
        s.println("OK");
      }
      if (strcmp(cmd.args[1], "reset_gyro") == 0 || strcmp(cmd.args[1], "reset_all") == 0)
      {
        _model.state.gyro.bias = {};
        s.println("OK");
      }
      if (strcmp(cmd.args[1], "reset_mag") == 0 || strcmp(cmd.args[1], "reset_all") == 0)
      {
        _model.state.mag.calibrationOffset = {};
        _model.state.mag.calibrationScale = {1.f, 1.f, 1.f};
        s.println("OK");
      }
    }
  }
  else if (strcmp(cmd.args[0], "gps") == 0)
  {
    if (cmd.args[1] && strcmp(cmd.args[1], "set_home") == 0)
    {
      _model.setGpsHome(true);
      s.println(_model.state.gps.homeSet ? "Home position set" : "No GPS fix");
    }
    else if (cmd.args[1] && strcmp(cmd.args[1], "clear_home") == 0)
    {
      _model.state.gps.homeSet = false;
      s.println("Home position cleared");
    }
    else
    {
      printGpsStatus(s, true);
    }
  }
  else if (strcmp(cmd.args[0], "preset") == 0)
  {
    if (!cmd.args[1])
    {
      s.println("Available presets: scaler, modes, micrus, brobot");
    }
    else if (strcmp(cmd.args[1], "scaler") == 0)
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

      s.println("OK");
    }
    else if (strcmp(cmd.args[1], "modes") == 0)
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

      s.println("OK");
    }
    else if (strcmp(cmd.args[1], "micrus") == 0)
    {
      s.println("OK");
    }
    else if (strcmp(cmd.args[1], "brobot") == 0)
    {
      s.println("OK");
    }
    else
    {
      s.println("NOT OK");
    }
  }
  else if (strcmp(cmd.args[0], "load") == 0)
  {
    _model.load();
    s.println("OK");
  }
  else if (strcmp(cmd.args[0], "save") == 0)
  {
    _model.save();
    s.println("# Saved, type reboot to apply changes");
    s.println();
  }
  else if (strcmp(cmd.args[0], "eeprom") == 0)
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
  else if (strcmp(cmd.args[0], "scaler") == 0)
  {
    for (size_t i = 0; i < SCALER_COUNT; i++)
    {
      uint32_t mode = _model.config.scaler[i].dimension;
      if (!mode) continue;
      short c = _model.config.scaler[i].channel;
      float v = _model.state.input.ch[c];
      float min = _model.config.scaler[i].minScale * 0.01f;
      float max = _model.config.scaler[i].maxScale * 0.01f;
      float scale = Utils::map3(v, -1.f, 0.f, 1.f, min, min < 0 ? 0.f : 1.f, max);
      s.print("scaler: ");
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
  else if (strcmp(cmd.args[0], "mixer") == 0)
  {
    const MixerConfig& mixer = _model.state.currentMixer;
    s.print("set mix_outputs ");
    s.println(mixer.count);
    Param p;
    for (size_t i = 0; i < MIXER_RULE_MAX; i++)
    {
      s.print("set mix_");
      s.print(i);
      s.print(' ');
      p.print(s, mixer.mixes[i]);
      s.println();
      if (mixer.mixes[i].src == MIXER_SOURCE_NULL) break;
    }
  }
  else if (strcmp(cmd.args[0], "status") == 0)
  {
    printVersion(s);
    s.println();
    s.println("STATUS: ");
    printStats(s);
    s.println();

    Device::GyroDevice* gyro = _model.state.gyro.dev;
    Device::BaroDevice* baro = _model.state.baro.dev;
    Device::MagDevice* mag = _model.state.mag.dev;
    s.print("     devices: ");
    if (gyro)
    {
      s.print(Device::GyroDevice::getName(gyro->getType()));
      s.print('/');
      s.print(Device::BusDevice::getName(gyro->getBus()->getType()));
    }
    else
    {
      s.print("NO GYRO");
    }

    if (baro)
    {
      s.print(", ");
      s.print(Device::BaroDevice::getName(baro->getType()));
      s.print('/');
      s.print(Device::BusDevice::getName(baro->getBus()->getType()));
    }

    if (mag)
    {
      s.print(", ");
      s.print(Device::MagDevice::getName(mag->getType()));
      s.print('/');
      s.print(Device::BusDevice::getName(mag->getBus()->getType()));
    }

    if (_model.state.gps.present)
    {
      s.print(", GPS");
    }
    s.println();

    const auto gRate = _model.state.gyro.timer.rate;
    const auto lRate = _model.state.loopTimer.rate;
    const auto aRate = _model.state.accel.timer.rate;

    float gyroDelay = Utils::estimateFilterDelay(_model.config.gyro.filter, lRate) +
                      Utils::estimateFilterDelay(_model.config.gyro.filter2, lRate) +
                      Utils::estimateFilterDelay(_model.config.gyro.filter3, gRate);
    float imuDelay =
        gyroDelay + Utils::estimateFilterDelay(FilterConfig(FILTER_PT1, aRate / GYRO_FUSION_LPF_DIV), aRate);
    float accelDelay = Utils::estimateFilterDelay(_model.config.accel.filter, aRate);
    float qDelay = Utils::estimateFilterDelay(FilterConfig(FILTER_BIQUAD, 20), aRate);

    s.print("     filters: ");
    s.print("gyro: ");
    s.print(gyroDelay * 1000.f, 1);
    s.print(" ms, accel: ");
    s.print(accelDelay * 1000.f, 1);
    s.print(" ms, imu: ");
    s.print(imuDelay * 1000.f, 1);
    s.print(" ms, q: ");
    s.print(qDelay * 1000.f, 1);
    s.println();

    s.print("       input: ");
    s.print(_model.state.input.frameRate);
    s.print(" Hz, ");
    s.print(_model.state.input.autoFreq);
    s.print(" Hz, ");
    s.println(_model.state.input.autoFactor);

    // clang-format off
    static const char* armingDisableNames[] = {
      "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "BAD_RX_RECOVERY", "BOXFAILSAFE", "RUNAWAY_TAKEOFF", "CRASH_DETECTED",
      "THROTTLE", "ANGLE", "BOOT_GRACE_TIME", "NOPREARM", "LOAD", "CALIBRATING", "CLI", "CMS_MENU", "BST",
      "MSP", "PARALYZE", "GPS", "RESC", "RPMFILTER", "REBOOT_REQUIRED", "DSHOT_BITBANG", "ACC_CALIBRATION",
      "MOTOR_PROTOCOL", "CRASHFLIP", "ALTHOLD", "POSHOLD", "AUTOPILOT", "ARM_SWITCH"
    };
    // clang-format on
    constexpr size_t armingDisableNamesLength = std::size(armingDisableNames);
    static_assert(armingDisableNamesLength == ARMING_DISABLED_FLAGS_COUNT,
                  "armingDisableNamesLength != ARMING_DISABLED_FLAGS_COUNT");

    s.print("   arm flags:");
    for (size_t i = 0; i < armingDisableNamesLength; i++)
    {
      if (_model.state.mode.armingDisabledFlags & (1 << i))
      {
        s.print(' ');
        s.print(armingDisableNames[i]);
      }
    }
    s.println();
    s.print(" rescue mode: ");
    s.print(_model.state.mode.rescueConfigMode);
    s.println();

    s.print("      uptime: ");
    s.print(millis() * 0.001, 1);
    s.println();
  }
  else if (strcmp(cmd.args[0], "stats") == 0)
  {
    printVersion(s);
    s.println();
    printStats(s);
    s.println();
    for (int i = 0; i < COUNTER_COUNT; ++i)
    {
      StatCounter c = (StatCounter)i;
      int time = lrintf(_model.state.stats.getTime(c));
      float load = _model.state.stats.getLoad(c);
      int freq = lrintf(_model.state.stats.getFreq(c));
      int real = lrintf(_model.state.stats.getReal(c));
      if (freq == 0) continue;

      s.print(_model.state.stats.getName(c));
      s.print(": ");
      if (time < 100) s.print(' ');
      if (time < 10) s.print(' ');
      s.print(time);
      s.print("us,  ");

      if (real < 100) s.print(' ');
      if (real < 10) s.print(' ');
      s.print(real);
      s.print("us/i,  ");

      if (load < 10) s.print(' ');
      s.print(load, 1);
      s.print("%,  ");

      if (freq < 1000) s.print(' ');
      if (freq < 100) s.print(' ');
      if (freq < 10) s.print(' ');
      s.print(freq);
      s.print(" Hz");
      s.println();
    }
    s.print("  TOTAL: ");
    s.print((int)(_model.state.stats.getCpuTime()));
    s.print("us, ");
    s.print(_model.state.stats.getCpuLoad(), 1);
    s.print("%");
    s.println();
  }
  else if (strcmp(cmd.args[0], "reboot") == 0 || strcmp(cmd.args[0], "exit") == 0)
  {
    _active = false;
    _interactive = false;
    Hardware::restart(_model);
  }
  else if (strcmp(cmd.args[0], "defaults") == 0)
  {
    _model.reset();
  }
  else if (strcmp(cmd.args[0], "motors") == 0)
  {
    s.print("count: ");
    s.println(getMotorCount());
    for (size_t i = 0; i < 8; i++)
    {
      s.print(i);
      s.print(": ");
      if (i >= OUTPUT_CHANNELS || _model.config.pin[i + PIN_OUTPUT_0] == -1)
      {
        s.print(-1);
        s.print(' ');
        s.println(0);
      }
      else
      {
        s.print(_model.config.pin[i + PIN_OUTPUT_0]);
        s.print(' ');
        s.println(_model.state.output.us[i]);
      }
    }
  }
  else if (strcmp(cmd.args[0], "logs") == 0)
  {
    s.print(_model.logger.c_str());
    s.print("usage: ");
    s.println(_model.logger.length());
  }
#ifdef USE_FLASHFS
  else if (strcmp(cmd.args[0], "flash") == 0)
  {
    if (!cmd.args[1])
    {
      size_t total = flashfsGetSize();
      size_t used = flashfsGetOffset();
      s.printf("total: %zu\r\n", total);
      s.printf(" used: %zu\r\n", used);
      s.printf(" free: %zu\r\n", total - used);
    }
    else if (strcmp(cmd.args[1], "partitions") == 0)
    {
      Device::FlashDevice::partitions(s);
    }
    else if (strcmp(cmd.args[1], "journal") == 0)
    {
      const FlashfsRuntime* flashfs = flashfsGetRuntime();
      FlashfsJournalItem journal[16];
      flashfsJournalLoad(journal, 0, 16);
      for (size_t i = 0; i < 16; i++)
      {
        const auto& it = journal[i];
        const auto& itr = flashfs->journal[i];
        s.printf("%02zu: %08X : %08X / %08X : %08X\r\n", i, it.logBegin, it.logEnd, itr.logBegin, itr.logEnd);
      }
      s.printf("current: %u\r\n", flashfs->journalIdx);
    }
    else if (strcmp(cmd.args[1], "erase") == 0)
    {
      flashfsEraseCompletely();
      s.println("OK");
    }
    else if (strcmp(cmd.args[1], "test") == 0)
    {
      const char* data = "flashfs-test";
      flashfsWrite((const uint8_t*)data, strlen(data), true);
      flashfsFlushAsync(true);
      flashfsClose();
      s.println("OK");
    }
    else if (strcmp(cmd.args[1], "print") == 0)
    {
      size_t addr = 0;
      if (cmd.args[2])
      {
        addr = String(cmd.args[2]).toInt();
      }
      size_t size = 0;
      if (cmd.args[3])
      {
        size = String(cmd.args[3]).toInt();
      }
      size = std::clamp<size_t>(size, 8u, 128 * 1024u);
      size_t chunk_size = 256;

      uint8_t* data = new uint8_t[chunk_size];
      while (size)
      {
        size_t len = std::min(size, chunk_size);
        flashfsReadAbs(addr, data, len);
        s.write(data, len);

        if (size > chunk_size)
        {
          size -= chunk_size;
          addr += chunk_size;
        }
        else
          break;
      }
      s.println();
      delete[] data;
    }
    else
    {
      s.println("wrong param!");
    }
  }
#endif
  else
  {
    s.print("unknown command: ");
    s.println(cmd.args[0]);
  }
  s.println();
}

void Cli::print(const Param& param, Stream& s) const
{
  s.print("set ");
  s.print(param.name);
  s.print(' ');
  param.print(s);
  s.println();
}

static constexpr const char* const gnssNames[] = {" GPS", "SBAS", "GALI", "BEID", "IMES", "QZSS", "GLON"};
static constexpr const char* const qualityNames[] = {"no_signal", "searching",    "acquired",     "unusable",
                                                     "locked",    "fully_locked", "fully_locked", "fully_locked"};
static constexpr const char* const usedNames[] = {" No", "Yes"};

static const char* const getGnssName(size_t num)
{
  constexpr size_t gnssNamesMax = sizeof(gnssNames) / sizeof(gnssNames[0]);
  if (num < gnssNamesMax) return gnssNames[num];
  return "?";
}

static const char* const getQualityName(size_t num)
{
  constexpr size_t qualityNamesMax = sizeof(qualityNames) / sizeof(qualityNames[0]);
  if (num < qualityNamesMax) return qualityNames[num];
  return "?";
}

static const char* const getUsedName(size_t num)
{
  constexpr size_t usedNamesMax = sizeof(usedNames) / sizeof(usedNames[0]);
  if (num < usedNamesMax) return usedNames[num];
  return "?";
}

void Cli::printGpsStatus(Stream& s, bool full) const
{
#ifndef UNIT_TEST
  s.println("GPS STATUS:");

  s.print("   Fix: ");
  s.print(_model.state.gps.fix);
  s.print(" (");
  s.print(_model.state.gps.fixType);
  s.println(")");

  s.print("   Lat: ");
  s.print(_model.state.gps.location.raw.lat);
  s.print(" (");
  s.print(_model.state.gps.location.raw.lat * 1e-7f, 7);
  s.print(" deg)");
  s.println();

  s.print("   Lon: ");
  s.print(_model.state.gps.location.raw.lon);
  s.print(" (");
  s.print(_model.state.gps.location.raw.lon * 1e-7f, 7);
  s.print(" deg)");
  s.println();

  s.print("Height: ");
  s.print(_model.state.gps.location.raw.height);
  s.print(" (");
  s.print(_model.state.gps.location.raw.height * 0.001f);
  s.print(" m)");
  s.println();

  s.print(" Speed: ");
  s.print(_model.state.gps.velocity.raw.groundSpeed);
  s.print(" (");
  s.print(_model.state.gps.velocity.raw.groundSpeed * 0.001f);
  s.print(" m/s, ");
  s.print(_model.state.gps.velocity.raw.groundSpeed * 0.0036f);
  s.print(" km/h)");
  s.println();

  s.print("  Head: ");
  s.print(_model.state.gps.velocity.raw.heading);
  s.print(" (");
  s.print(_model.state.gps.velocity.raw.heading * 0.00001f);
  s.print(" deg)");
  s.println();

  s.print("  Accu: ");
  s.print(_model.state.gps.accuracy.horizontal * 0.001f);
  s.print(" m, ");
  s.print(_model.state.gps.accuracy.vertical * 0.001f);
  s.print(" m, ");
  s.print(_model.state.gps.accuracy.speed * 0.001f);
  s.print(" m/s, ");
  s.print(_model.state.gps.accuracy.heading * 0.00001f);
  s.print(" deg, pDOP: ");
  s.print(_model.state.gps.accuracy.pDop * 0.01f);
  s.println();

  const GpsDateTime& gdt = _model.state.gps.dateTime;
  s.printf("  Time: %04d-%02d-%02d %02d:%02d:%02d.%03d UTC", gdt.year, gdt.month, gdt.day, gdt.hour, gdt.minute,
           gdt.second, gdt.msec);
  s.println();

  s.print("  Rate: ");
  s.print(1000000.0f / _model.state.gps.interval, 1);
  s.println(" Hz");

  s.print("  Sats: ");
  s.print(_model.state.gps.numSats);
  s.print(" (");
  s.print(_model.state.gps.numCh);
  s.println(" ch)");

  s.printf("GNSS  ID Sig Used Quality");
  s.println();
  for (size_t i = 0; i < _model.state.gps.numCh; i++)
  {
    const GpsSatelite& sv = _model.state.gps.svinfo[i];
    s.printf("%s %3d %3d  %s %s", getGnssName(sv.gnssId), sv.id, sv.cno, getUsedName(sv.quality.svUsed),
             getQualityName(sv.quality.qualityInd));
    s.println();
  }
  s.println("Home:");
  if (_model.state.gps.homeSet)
  {
    s.print("  Lat:  ");
    s.print(_model.state.gps.location.home.lat);
    s.print(" (");
    s.print(_model.state.gps.location.home.lat * 1e-7f, 7);
    s.println(")");

    s.print("  Lon:  ");
    s.print(_model.state.gps.location.home.lon);
    s.print(" (");
    s.print(_model.state.gps.location.home.lon * 1e-7f, 7);
    s.println(")");

    s.print("  Dist: ");
    s.print(Utils::toDeg(_model.state.gps.distanceToHome), 2);
    s.println(" m");

    s.print("  Bear: ");
    s.print(Utils::toDeg(_model.state.gps.directionToHome), 2);
    s.println(" deg");
  }
  else
  {
    s.println("  Not set");
  }
#endif
}

void Cli::printVersion(Stream& s) const
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
  s.print(" api=");
  s.print(API_VERSION_MAJOR);
  s.print('.');
  s.print(API_VERSION_MINOR);
  s.print(" gcc=");
  s.print(__VERSION__);
  s.print(" std=");
  s.print(__cplusplus);
}

void Cli::printStats(Stream& s) const
{
  s.print("    cpu freq: ");
  s.print(targetCpuFreq());
  s.println(" MHz");

  s.print("  gyro clock: ");
  s.print(_model.state.gyro.clock);
  s.println(" Hz");

  s.print("   gyro rate: ");
  s.print(_model.state.gyro.timer.rate);
  s.println(" Hz");

  s.print("   loop rate: ");
  s.print(_model.state.loopTimer.rate);
  s.println(" Hz");

  s.print("  mixer rate: ");
  s.print(_model.state.mixer.timer.rate);
  s.println(" Hz");

  s.print("  accel rate: ");
  s.print(_model.state.accel.timer.rate);
  s.println(" Hz");

  s.print("   baro rate: ");
  s.print(_model.state.baro.rate);
  s.println(" Hz");

  s.print("    mag rate: ");
  s.print(_model.state.mag.timer.rate);
  s.println(" Hz");
}

} // namespace Connect

} // namespace Espfc
