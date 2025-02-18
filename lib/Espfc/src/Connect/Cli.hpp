#pragma once

#include "Model.h"

namespace Espfc {

namespace Connect {

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
    PARAM_INPUT_CHANNEL,   //  input channel config
    PARAM_OUTPUT_CHANNEL,  // output channel config
    PARAM_SCALER,  // scaler config
    PARAM_MODE,    // scaler config
    PARAM_MIXER,   // mixer config
    PARAM_SERIAL,  // mixer config
    PARAM_STRING,  // string
    PARAM_BITMASK, // set or clear bit
  };

  class Param
  {
    public:
      Param(): Param(NULL, PARAM_NONE, NULL, NULL) {}
      Param(const Param& p): Param(p.name, p.type, p.addr, p.choices) {}

      Param(const char * n, ParamType t, char * a, const char * const * c, size_t l = 16): name(n), type(t), addr(a), choices(c), maxLen(l) {}

      Param(const char * n, bool    * a): Param(n, PARAM_BOOL,   reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, int8_t  * a): Param(n, PARAM_BYTE,   reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, uint8_t * a): Param(n, PARAM_BYTE_U, reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, int16_t * a): Param(n, PARAM_SHORT,  reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, int32_t * a): Param(n, PARAM_INT,    reinterpret_cast<char*>(a), NULL) {}

      Param(const char * n, int8_t * a, const char * const * c): Param(n, PARAM_BYTE, reinterpret_cast<char*>(a), c) {}
      Param(const char * n, int32_t * a, uint8_t b):  Param(n, PARAM_BITMASK,  reinterpret_cast<char*>(a), NULL, b) {}

      Param(const char * n, InputChannelConfig * a):  Param(n, PARAM_INPUT_CHANNEL,  reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, OutputChannelConfig * a): Param(n, PARAM_OUTPUT_CHANNEL, reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, ScalerConfig * a):        Param(n, PARAM_SCALER, reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, ActuatorCondition * a):   Param(n, PARAM_MODE,   reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, MixerEntry * a):          Param(n, PARAM_MIXER,  reinterpret_cast<char*>(a), NULL) {}
      Param(const char * n, SerialPortConfig * a):    Param(n, PARAM_SERIAL, reinterpret_cast<char*>(a), NULL) {}

      void print(Stream& stream) const;
      void print(Stream& stream, const OutputChannelConfig& och) const;
      void print(Stream& stream, const InputChannelConfig& ich) const;
      void print(Stream& stream, const ScalerConfig& sc) const;
      void print(Stream& stream, const ActuatorCondition& ac) const;
      void print(Stream& stream, const MixerEntry& me) const;
      void print(Stream& stream, const SerialPortConfig& sc) const;
      void print(Stream& stream, int32_t v) const;

      void update(const char ** args) const;

      void write(OutputChannelConfig& och, const char ** args) const;
      void write(InputChannelConfig& ich, const char ** args) const;
      void write(ScalerConfig& sc, const char ** args) const;
      void write(ActuatorCondition& ac, const char ** args) const;
      void write(MixerEntry& ac, const char ** args) const;
      void write(SerialPortConfig& sc, const char ** args) const;

      template<typename T>
      void write(const T v) const
      {
        *reinterpret_cast<T*>(addr) = v;
      }

      void write(const String& v) const;
      int32_t parse(const char * v) const;

      const char * name;
      ParamType type;
      char * addr;
      const char * const * choices;
      size_t maxLen;
  };

  Cli(Model& model);
  static const Param * initialize(ModelConfig& c);
  bool process(const char c, CliCmd& cmd, Stream& stream);
  void parse(CliCmd& cmd);
  void execute(CliCmd& cmd, Stream& s);

private:
  void print(const Param& param, Stream& s) const;
  void printGpsStatus(Stream& s, bool full) const;
  void printVersion(Stream& s) const;
  void printStats(Stream& s) const;

  Model& _model;
  const Param * _params;
  bool _ignore;
  bool _active;
};

}

}
