#ifndef _ESPFC_LOGGER_H_
#define _ESPFC_LOGGER_H_

#include "Arduino.h"
#include "FS.h"

namespace Espfc {

class Logger
{
  public:
    int begin()
    {
      _valid = false;
      SPIFFS.begin();
      for(size_t i = 1; i < 100; i++)
      {
        String name;
        _mkname(name, i);
        if(!SPIFFS.exists(name))
        {
          _name = name;
          _valid = true;
          //_file = SPIFFS.open(name, "a+");
          break;
        }
      }
      info().logln(F("init"));
      return 1;
    }

    void list(Stream * s)
    {
      if(!s) return;
      Dir dir = SPIFFS.openDir("");
      while(dir.next())
      {
        s->print(dir.fileName());
        s->print(' ');
        File f = dir.openFile("r");
        s->println(f.size());
      }
    }

    void show(Stream * s, int i)
    {
      if(!s) return;
      String name;
      _mkname(name, i);
      File f = SPIFFS.open(name, "r");
      if(!f)
      {
        s->println(F("Error reading file"));
        return;
      }
      while(f.available())
      {
        String line = f.readStringUntil('\n');
        s->println(line);
      }
    }

    bool format()
    {
      return SPIFFS.format();
    }

    void info(Stream * s)
    {
      if(!s) return;
      FSInfo i;
      SPIFFS.info(i);
      s->print(F("total: ")); s->println(i.totalBytes);
      s->print(F("used: ")); s->println(i.usedBytes);
      s->print(F("avail: ")); s->println(i.totalBytes - i.usedBytes);
      s->print(F("block: ")); s->println(i.blockSize);
      s->print(F("page: ")); s->println(i.pageSize);
      s->print(F("max files: ")); s->println(i.maxOpenFiles);
      s->print(F("max path: ")); s->println(i.maxPathLength);
    }

    Logger& info()
    {
      if(!_valid) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(millis());
        f.print(F(" INF "));
        f.close();
      }
      return *this;
    }

    Logger& err()
    {
      if(!_valid) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(millis());
        f.print(F(" ERR "));
        f.close();
      }
      return *this;
    }

    template<typename T>
    Logger& log(const T& v)
    {
      if(!_valid) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(v);
        f.print(' ');
        f.close();
      }
      return *this;
    }

    template<typename T>
    Logger& logln(const T& v)
    {
      if(!_valid) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.println(v);
        f.close();
      }
      return *this;
    }

    void _mkname(String& name, int i)
    {
      name = "";
      if(i < 10) name += "00";
      else if(i < 100) name += "0";
      name += i;
    }

    String _name;
    File _file;
    bool _valid;
};

}

#endif
