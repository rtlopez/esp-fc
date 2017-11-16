#ifndef _ESPFC_LOGGER_H_
#define _ESPFC_LOGGER_H_

#include "Arduino.h"
#include "FS.h"

#define LOG_SERIAL_DEBUG(v)
//#define LOG_SERIAL_DEBUG(v) Serial.println(v)

namespace Espfc {

class Logger
{
  public:
    int begin()
    {
      _valid = false;
      SPIFFS.begin();
      for(size_t i = 1; i < 1000; i++)
      {
        String name;
        _mkname(name, i);
        if(!SPIFFS.exists(name))
        {
          _name = name;
          _valid = true;
          break;
        }
      }
      info().logln(F("INIT"));
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
      s->print(F("total: ")); s->print(i.totalBytes / 1024); s->println(F(" kB"));
      s->print(F(" used: ")); s->print(i.usedBytes / 1024); s->println(F(" kB"));
      s->print(F("avail: ")); s->print((i.totalBytes - i.usedBytes) / 1024); s->println(F(" kB"));
      s->print(F("block: ")); s->println(i.blockSize);
      s->print(F(" page: ")); s->println(i.pageSize);
      s->print(F("files: ")); s->println(i.maxOpenFiles);
      s->print(F(" path: ")); s->println(i.maxPathLength);
    }

    Logger& info()
    {
      LOG_SERIAL_DEBUG("INF");
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(millis());
        f.print(F(" INF"));
        f.close();
      }
      return *this;
    }

    Logger& err()
    {
      LOG_SERIAL_DEBUG("ERR");
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(millis());
        f.print(F(" ERR"));
        f.close();
      }
      return *this;
    }

    template<typename T>
    Logger& log(const T& v)
    {
      LOG_SERIAL_DEBUG(v);
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(' ');
        f.print(v);
        f.close();
      }
      return *this;
    }

    template<typename T>
    Logger& logln(const T& v)
    {
      LOG_SERIAL_DEBUG(v);
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(' ');
        f.println(v);
        f.close();
      }
      return *this;
    }

    bool _available()
    {
      if(_valid)
      {
        FSInfo i;
        SPIFFS.info(i);
        _valid = i.totalBytes - i.usedBytes > 1024; // keep 1kB free space margin
      }
      return _valid;
    }

    void _mkname(String& name, int i)
    {
      name = "";
      if(i < 10) name += "000";
      else if(i < 100) name += "00";
      else if(i < 1000) name += "0";
      name += i;
    }

    String _name;
    File _file;
    bool _valid;
};

}

#endif
