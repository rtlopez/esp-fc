#ifndef _ESPFC_LOGGER_H_
#define _ESPFC_LOGGER_H_

#include <Arduino.h>
#include "Debug.h"

#if defined(ESP32)
#include "SPIFFS.h"
#elif defined(ESP8266) && false
#include "FS.h"
#define USE_FS_LOGGER
#endif

namespace Espfc {

class Logger
{
  public:
    int begin()
    {
      return 0;
      _valid = false;
#if defined(USE_FS_LOGGER)
      SPIFFS.begin();
      int count = 0;
      int first = 0;
      int last = 0;

      Dir dir = SPIFFS.openDir("");
      while(dir.next())
      {
        String fn = dir.fileName();
        int id = String(fn).toInt();
        if(!id) continue;

        last = max(last, id);
        if(!first) first = id;
        first = min(first, id);
        count++;
      }
      int next = last + 1;
      int remove = count > 20 && first;

      String name;
      if(remove)
      {
        _mkname(name, first);
        SPIFFS.remove(name);
      }

      _mkname(_name, next);
      _valid = true;
      info().logln(F("LOG INIT"));

      if(remove)
      {
        info().log(F("LOG RM")).logln(name);
      }
#endif
      return 1;
    }

    void list(Stream * s)
    {
      if(!s) return;
#if defined(USE_FS_LOGGER)
      Dir dir = SPIFFS.openDir("");
      while(dir.next())
      {
        s->print(dir.fileName());
        s->print(' ');
        File f = dir.openFile("r");
        s->println(f.size());
      }
#endif
    }

    void show(Stream * s, int i)
    {
      String name;
      _mkname(name, i);
      show(s, name);
    }

    void show(Stream * s)
    {
      show(s, _name);
    }

    void show(Stream * s, const String& name)
    {
      if(!s) return;
#if defined(USE_FS_LOGGER)
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
#endif
    }

    bool format()
    {
#if defined(USE_FS_LOGGER)
      return SPIFFS.format();
#else
      return false;
#endif
    }

    void info(Stream * s)
    {
      if(!s) return;
#if defined(USE_FS_LOGGER)
      FSInfo i;
      SPIFFS.info(i);
      s->print(F("total: ")); s->print(i.totalBytes / 1024); s->println(F(" kB"));
      s->print(F(" used: ")); s->print(i.usedBytes / 1024); s->println(F(" kB"));
      s->print(F("avail: ")); s->print((i.totalBytes - i.usedBytes) / 1024); s->println(F(" kB"));
      s->print(F("block: ")); s->println(i.blockSize);
      s->print(F(" page: ")); s->println(i.pageSize);
      s->print(F("files: ")); s->println(i.maxOpenFiles);
      s->print(F(" path: ")); s->println(i.maxPathLength);
#endif
    }

    Logger& info()
    {
      LOG_SERIAL_DEBUG("I")
#if defined(USE_FS_LOGGER)
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(millis());
        f.print(F(" I"));
        f.close();
      }
#endif
      return *this;
    }

    Logger& err()
    {
      LOG_SERIAL_DEBUG("E")
#if defined(USE_FS_LOGGER)
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(millis());
        f.print(F(" E"));
        f.close();
      }
#endif
      return *this;
    }

    template<typename T>
    Logger& log(const T& v)
    {
      LOG_SERIAL_DEBUG(v)
#if defined(USE_FS_LOGGER)
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(' ');
        f.print(v);
        f.close();
      }
#endif
      return *this;
    }

    template<typename T>
    Logger& logln(const T& v)
    {
      LOG_SERIAL_DEBUG(v)
      LOG_SERIAL_DEBUG('\r')
      LOG_SERIAL_DEBUG('\n')
#if defined(USE_FS_LOGGER)
      if(!_available()) return *this;
      File f = SPIFFS.open(_name, "a");
      if(f)
      {
        f.print(' ');
        f.println(v);
        f.close();
      }
#endif
      return *this;
    }

    bool _available()
    {
#if defined(USE_FS_LOGGER)
      if(_valid)
      {
        FSInfo i;
        SPIFFS.info(i);
        _valid = i.totalBytes - i.usedBytes > 1024; // keep 1kB free space margin
      }
      return _valid;
#else
      return false;
#endif
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
    //File _file;
    bool _valid;
};

}

#endif
