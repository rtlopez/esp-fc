#ifndef _ESPFC_CLI_H_
#define _ESPFC_CLI_H_

#include <cstring>
#include <cctype>

#include "Model.h"

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
      PARAM_FLOAT   // 32 bit float
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
            stream.print("UNDEF");
            return;
          }
          switch(type)
          {
            case PARAM_NONE:  stream.print("NONE"); break;
            case PARAM_BOOL:  stream.print(*addr != 0); break;
            case PARAM_BYTE:  stream.print((int)(*addr)); break;
            case PARAM_SHORT: stream.print(*reinterpret_cast<short*>(addr)); break;
            case PARAM_INT:   stream.print(*reinterpret_cast<int*>(addr)); break;
            case PARAM_FLOAT: stream.print(*reinterpret_cast<float*>(addr), 4); break;
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
          }
        }
        const char * name;
        char * addr;
        ParamType type;
    };

    Cli(Model& model, Stream& stream): _model(model), _stream(stream), _index(0)
    {
      ModelConfig * c = &_model.config;
      size_t i = 0;
      _params[i++] = Param(PSTR("telemetry"), (char*)&c->telemetry, PARAM_BOOL);
      _params[i++] = Param(PSTR("telemetry_interval"), (char*)&c->telemetryInterval, PARAM_INT);
      _params[i++] = Param(PSTR("gyro_fifo"), (char*)&c->gyroFifo, PARAM_BOOL);
      _params[i++] = Param(PSTR("gyro_rate"), (char*)&c->gyroSampleRate, PARAM_INT);
      _params[i++] = Param(PSTR("compass_rate"), (char*)&c->magSampleRate, PARAM_INT);
      _params[i++] = Param(PSTR("compass_calibration"), (char*)&c->magCalibration, PARAM_BOOL);
      _params[i++] = Param(PSTR("rate_pitch_max"), (char*)&c->rateMax[AXIS_PITH], PARAM_FLOAT);
      _params[i++] = Param(PSTR("rate_roll_max"), (char*)&c->rateMax[AXIS_ROLL], PARAM_FLOAT);
      _params[i++] = Param(PSTR("rate_yaw_max"), (char*)&c->rateMax[AXIS_YAW], PARAM_FLOAT);
      _params[i++] = Param(PSTR("angle_pitch_max"), (char*)&c->angleMax[AXIS_PITH], PARAM_FLOAT);
      _params[i++] = Param(PSTR("angle_roll_max"), (char*)&c->angleMax[AXIS_ROLL], PARAM_FLOAT);
    }

    int begin()
    {
    }

    int update()
    {
      while(_stream.available() > 0)
      {
        process(_stream.read());
      }
    }

    void process(char c)
    {
      if(c == '\r') return;
      if(c == '\n' || _index >= BUFF_SIZE)
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
        _stream.print(_cmd.args[i]);
        _stream.print(' ');
      }
      _stream.println();

      if(!_cmd.args[0]) return;

      if(strcmp_P(_cmd.args[0], PSTR("help")) == 0)
      {
        _stream.println(F("available commands:\n help\n list\n get param\n set param value\n load\n save\n info\n version"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("version")) == 0)
      {
        _stream.println(F("0.1"));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("info")) == 0)
      {
        _stream.print(F(" bool: ")); _stream.println(sizeof(bool));
        _stream.print(F(" char: ")); _stream.println(sizeof(char));
        _stream.print(F("short: ")); _stream.println(sizeof(short));
        _stream.print(F("  int: ")); _stream.println(sizeof(int));
        _stream.print(F(" long: ")); _stream.println(sizeof(long));
        _stream.print(F("float: ")); _stream.println(sizeof(float));
      }
      else if(strcmp_P(_cmd.args[0], PSTR("get")) == 0)
      {
        if(!_cmd.args[1])
        {
          _stream.println(F("param required"));
          return;
        }
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;

          if(strcmp_P(_cmd.args[1], _params[i].name) == 0)
          {
            _stream.print(FPSTR(_params[i].name));
            _stream.print(" = ");
            _params[i].print(_stream);
            _stream.println();
            return;
          }
        }
        _stream.print(F("param not found: "));
        _stream.println(_cmd.args[1]);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("set")) == 0)
      {
        if(!_cmd.args[1])
        {
          _stream.println(F("param required"));
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
        _stream.print(F("param not found: "));
        _stream.println(_cmd.args[1]);
      }
      else if(strcmp_P(_cmd.args[0], PSTR("list")) == 0)
      {
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;
          print(_params[i]);
        }
      }
      else
      {
        _stream.print(F("command not found: "));
        _stream.println(_cmd.args[0]);
      }
      _stream.println();
    }

  private:
    void print(const Param& param)
    {
      _stream.print(FPSTR(param.name));
      _stream.print(" = ");
      param.print(_stream);
      _stream.println();
    }

    static const size_t PARAM_SIZE = 16;
    static const size_t BUFF_SIZE = 64;

    Model& _model;
    Stream& _stream;
    Param _params[PARAM_SIZE];
    char _buff[BUFF_SIZE];
    size_t _index;
    Cmd _cmd;
};

}

#endif
