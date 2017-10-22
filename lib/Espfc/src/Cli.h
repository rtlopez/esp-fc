#ifndef _ESPFC_CLI_H_
#define _ESPFC_CLI_H_

#include <cstring>
#include <cctype>

extern "C" {
#include "user_interface.h"
}

#include "Model.h"
#include "Msp.h"
#include "Hardware.h"

namespace Espfc {

class Cli
{
  public:
    class Cmd
    {
      public:
        Cmd() { for(size_t i = 0; i < ARGS_SIZE; ++i) args[i] = NULL; }
        static const size_t ARGS_SIZE = 8;
        const char * args[ARGS_SIZE];
    };

    enum ParamType {
      PARAM_NONE,   // unused
      PARAM_BOOL,   // boolean
      PARAM_BYTE,   // 8 bit int
      PARAM_SHORT,  // 16 bit int
      PARAM_INT,    // 32 bit int
      PARAM_FLOAT,   // 32 bit float
      PARAM_VECTOR_FLOAT   // 32 bit float vector
    };

    class Param
    {
      public:
        Param(): name(NULL), addr(NULL), type(PARAM_NONE) {}
        Param(const char * n, char * a, ParamType t): name(n), addr(a), type(t) {}
        Param(const Param& c): name(c.name), addr(c.addr), type(c.type) {}

        void print(Stream& stream) const
        {
          if(!addr)
          {
            stream.print(F("UNSET"));
            return;
          }
          switch(type)
          {
            case PARAM_NONE:  stream.print("NONE"); break;
            case PARAM_BOOL:  stream.print(*addr != 0); break;
            case PARAM_BYTE:  stream.print((int)(*addr)); break;
            case PARAM_SHORT: stream.print(*reinterpret_cast<short*>(addr)); break;
            case PARAM_INT:   stream.print(*reinterpret_cast<long*>(addr)); break;
            case PARAM_FLOAT: stream.print(*reinterpret_cast<float*>(addr), 4); break;
            case PARAM_VECTOR_FLOAT:
              {
                const VectorFloat * v = reinterpret_cast<VectorFloat*>(addr);
                stream.print(v->x, 4); stream.print(' '); stream.print(v->y, 4); stream.print(' '); stream.print(v->z, 4);
              }
              break;
          }
        }

        void update(const char * v)
        {
          if(!addr || !v) return;
          switch(type)
          {
            case PARAM_NONE:  break;
            case PARAM_BOOL:
              if(*v == '0') *addr = 0;
              if(*v == '1') *addr = 1;
              break;
            case PARAM_BYTE:
            case PARAM_SHORT:
            case PARAM_INT:
              {
                String tmp = v;
                *addr = tmp.toInt();
              }
              break;
            case PARAM_FLOAT:
              {
                String tmp = v;
                *addr = tmp.toFloat();
              }
              break;
            case PARAM_VECTOR_FLOAT:
              break;
          }
        }
        const char * name;
        char * addr;
        ParamType type;
    };

    Cli(Model& model): _model(model), _index(0), _msp(model)
    {
      ModelConfig * c = &_model.config;
      ModelState * s = &_model.state;
      size_t i = 0;
      _params[i++] = Param(PSTR("telemetry"), (char*)&c->telemetry, PARAM_BOOL);
      _params[i++] = Param(PSTR("telemetry_interval"), (char*)&c->telemetryInterval, PARAM_INT);
      _params[i++] = Param(PSTR("accel_mode"), (char*)&c->accelMode, PARAM_BYTE);
      _params[i++] = Param(PSTR("gyro_rate"), (char*)&c->gyroSampleRate, PARAM_BYTE);
      _params[i++] = Param(PSTR("compass_rate"), (char*)&c->magSampleRate, PARAM_BYTE);
      _params[i++] = Param(PSTR("compass_calibration"), (char*)&s->magCalibration, PARAM_BYTE);
      _params[i++] = Param(PSTR("compass_calibration_offset"), (char*)&c->magCalibrationOffset, PARAM_VECTOR_FLOAT);
      _params[i++] = Param(PSTR("compass_calibration_scale"), (char*)&c->magCalibrationScale, PARAM_VECTOR_FLOAT);
      _params[i++] = Param(PSTR("angle_max"), (char*)&c->angleMax, PARAM_BYTE);
    }

    int begin()
    {
      _stream = (Stream*)Hardware::getSerialPort((SerialPort)_model.config.cliPort);
      return 1;
    }

    int update()
    {
      if(!_stream) return 0;

      while((*_stream).available() > 0)
      {
        char c = (*_stream).read();
        if(!_msp.process(c, *_stream))
        {
          process(c);
        }
      }
    }

    void process(char c)
    {
      if(c == '\r') return;
      if(c == '\n' || _index >= BUFF_SIZE - 1)
      {
        parse();
        execute();
        _index = 0;
        _buff[_index] = '\0';
        return;
      }
      _buff[_index++] = c;
      _buff[_index] = '\0';
      return;
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
        println(F("available commands:\n help\n list\n get param\n set param value\n load\n save\n eeprom\n reset\n stats\n info\n version"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("version")) == 0)
      {
        println(F("0.1"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("info")) == 0)
      {
        print(F(" bool: ")); println(sizeof(bool));
        print(F(" char: ")); println(sizeof(char));
        print(F("short: ")); println(sizeof(short));
        print(F("  int: ")); println(sizeof(int));
        print(F(" long: ")); println(sizeof(long));
        print(F("float: ")); println(sizeof(float));
        print(F("model: ")); println(sizeof(ModelConfig));
        println();

        const rst_info * resetInfo = system_get_rst_info();
        print(F("system_get_rst_info() reset reason: "));
        println(resetInfo->reason);

        print(F("system_get_free_heap_size(): "));
        println(system_get_free_heap_size());

        print(F("system_get_os_print(): "));
        println(system_get_os_print());

        //system_print_meminfo();

        print(F("system_get_chip_id(): 0x"));
        println(system_get_chip_id(), HEX);

        print(F("system_get_sdk_version(): "));
        println(system_get_sdk_version());

        print(F("system_get_boot_version(): "));
        println(system_get_boot_version());

        print(F("system_get_userbin_addr(): 0x"));
        println(system_get_userbin_addr(), HEX);

        print(F("system_get_boot_mode(): "));
        println(system_get_boot_mode() == 0 ? F("SYS_BOOT_ENHANCE_MODE") : F("SYS_BOOT_NORMAL_MODE"));

        print(F("system_get_cpu_freq(): "));
        println(system_get_cpu_freq());

        print(F("system_get_flash_size_map(): "));
        println(system_get_flash_size_map());

        print(F("system_get_time(): "));
        println(system_get_time() / 1000000);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("get")) == 0)
      {
        if(!_cmd.args[1])
        {
          println(F("param required"));
          return;
        }
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;

          if(strcmp_P(_cmd.args[1], _params[i].name) == 0)
          {
            print(FPSTR(_params[i].name));
            print(" = ");
            _params[i].print(*_stream);
            println();
            return;
          }
        }
        print(F("param not found: "));
        println(_cmd.args[1]);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("set")) == 0)
      {
        if(!_cmd.args[1])
        {
          println(F("param required"));
          return;
        }
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;

          if(strcmp_P(_cmd.args[1], _params[i].name) == 0)
          {
            _params[i].update(_cmd.args[2]);
            print(_params[i]);
            return;
          }
        }
        print(F("param not found: "));
        println(_cmd.args[1]);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("list")) == 0)
      {
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;
          print(_params[i]);
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("mag")) == 0)
      {
        if(!_cmd.args[1]) {}
        else if(_cmd.args[1][0] == '1')
        {
          _model.state.magCalibration = 1;
          //_model.config.telemetry = 1;
          //_model.config.telemetryInterval = 200;
          print("mag calibration on");
        }
        else if(_cmd.args[1][0] == '0')
        {
          _model.state.magCalibration = 0;
          //_model.config.telemetry = 0;
          print("mag calibration off");
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("load")) == 0)
      {
        _model.load();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("save")) == 0)
      {
        _model.save();
      }
      else if(strcmp_P(_cmd.args[0], PSTR("eeprom")) == 0)
      {
        for(int i = 0; i < 32; ++i)
        {
          print(EEPROM.read(i), HEX);
          print(' ');
        }
      }
      else if(strcmp_P(_cmd.args[0], PSTR("stats")) == 0)
      {
        for(size_t i = 0; i < COUNTER_COUNT; ++i)
        {
          print(FPSTR(_model.state.stats.getName((StatCounter)i)));
          print(": ");
          print((int)_model.state.stats.getTime((StatCounter)i));
          print("us, ");
          print(_model.state.stats.getLoad((StatCounter)i), 1);
          print("%");
          println();
        }
        print(F("TOTAL       : "));
        print((int)_model.state.stats.getTotalTime());
        print(F("us, "));
        print(_model.state.stats.getTotalLoad(), 1);
        print(F("%"));
        println();
      }
      else
      {
        print(F("command not found: "));
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
      print(FPSTR(param.name));
      print(" = ");
      param.print(*_stream);
      println();
    }

    static const size_t PARAM_SIZE = 32;
    static const size_t BUFF_SIZE = 64;

    Model& _model;
    Stream * _stream;
    Param _params[PARAM_SIZE];
    char _buff[BUFF_SIZE];
    size_t _index;
    Cmd _cmd;
    Msp _msp;
};

}

#endif
