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
      PARAM_LONG,   // 64 bit int
      PARAM_FLOAT   // 32 bit float
    };

    class Param
    {
      public:
        Param(): name(NULL), addr(NULL), type(PARAM_NONE) {}
        Param(const char * n, char * a, ParamType t): name(n), addr(a), type(t) {}
        Param(const Param& c): name(c.name), addr(c.addr), type(c.type) {}
        void print(Stream& stream)
        {
          if(!addr)
          {
            stream.print("UNDEF");
            return;
          }
          switch(type)
          {
            case PARAM_NONE:  stream.print("NONE"); break;
            case PARAM_BOOL:  stream.print(*addr ? 0 : 1); break;
            case PARAM_BYTE:  stream.print((int)(*addr)); break;
            case PARAM_SHORT: stream.print(*reinterpret_cast<int16_t*>(addr)); break;
            case PARAM_INT:   stream.print(*reinterpret_cast<int*>(addr)); break;
            case PARAM_LONG:  stream.print(*reinterpret_cast<long*>(addr)); break;
            case PARAM_FLOAT: stream.print(*reinterpret_cast<float*>(addr), 4); break;
          }
        }
        const char * name;
        char * addr;
        ParamType type;
    };

    Cli(Model& model, Stream& stream): _model(model), _stream(stream), _index(0)
    {
      ModelConfig * c = &_model.config;
      _params[0] = Param("telemetry", (char*)&c->telemetry, PARAM_BOOL);
      _params[1] = Param("telemetry_interval", (char*)&c->telemetryInterval, PARAM_INT);
      _params[2] = Param("gyro_fifo", (char*)&c->gyroFifo, PARAM_BOOL);
      _params[3] = Param("gyro_rate", (char*)&c->gyroSampleRate, PARAM_INT);
      _params[4] = Param("compas_rate", (char*)&c->magSampleRate, PARAM_INT);
      _params[5] = Param("rate_pitch_max", (char*)&c->rateMax[AXIS_PITH], PARAM_FLOAT);
      _params[6] = Param("rate_roll_max", (char*)&c->rateMax[AXIS_ROLL], PARAM_FLOAT);
      _params[7] = Param("rate_yaw_max", (char*)&c->rateMax[AXIS_YAW], PARAM_FLOAT);
      _params[8] = Param("angle_pitch_max", (char*)&c->angleMax[AXIS_PITH], PARAM_FLOAT);
      _params[9] = Param("angle_roll_max", (char*)&c->angleMax[AXIS_ROLL], PARAM_FLOAT);
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
      if(!_cmd.args[0]) return;

      if(std::strcmp(_cmd.args[0], "help") == 0)
      {
        _stream.println("help message");
      }
      else if(std::strcmp(_cmd.args[0], "version") == 0)
      {
        _stream.println("0.1");
      }
      else if(std::strcmp(_cmd.args[0], "get") == 0)
      {
        if(_cmd.args[1])
        {
          _stream.println("param required");
          return;
        }
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;
          if(std::strcmp(_cmd.args[1], _params[i].name) == 0)
          {
            _params[i].print(_stream);
            _stream.println();
            return;
          }
        }
        _stream.print("param not found: ");
        _stream.println(_cmd.args[1]);
      }
      else if(std::strcmp(_cmd.args[0], "list") == 0)
      {
        for(size_t i = 0; i < PARAM_SIZE; ++i)
        {
          if(!_params[i].name) continue;
          _stream.print(_params[i].name);
          _stream.print(": ");
          _params[i].print(_stream);
          _stream.println();
        }
      }
      else
      {
        _stream.print("command not found: ");
        _stream.println(_cmd.args[0]);
      }
      _stream.println();
    }

  private:
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
