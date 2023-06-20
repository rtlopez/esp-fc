#ifndef _ESPFC_TELEMETRY_H_
#define _ESPFC_TELEMETRY_H_

#include "Model.h"
#include "Hardware.h"
#include "Rc/Crsf.h"

namespace Espfc {

class Telemetry
{
  public:
    Telemetry(Model& model): _model(model), _value(172), _up(1) {}

    int process(Stream& s) const
    {
      Stats::Measure measure(_model.state.stats, COUNTER_TELEMETRY);

      /*
      _value = _value + (_up ? 1 : -1);

      if (_up && _value >= 1811) _up = false;
      if (!_up && _value <= 172) _up = true;

      Rc::CrsfFrame frame;
      Rc::CrsfData data;
      data.chan0 = _value;
      data.chan1 = _value;
      data.chan2 = _value;
      data.chan3 = _value;
      data.chan4 = _value;
      data.chan5 = _value;
      data.chan6 = _value;
      data.chan7 = _value;
      data.chan8 = _value;
      data.chan9 = _value;
      data.chan10 = _value;
      data.chan11 = _value;
      data.chan12 = _value;
      data.chan13 = _value;
      data.chan14 = _value;
      data.chan15 = _value;

      Rc::Crsf::encodeRcData(frame, data);

      size_t size = frame.message.size + 2;
      s.write(frame.data, size);
      */
      //print(s, _model.state.gyro.x, 3);
      //println(s);

      return 1;
    }

  private:
    template<typename T>
    void print(Stream& s, const T& v) const
    {
      s.print(v);
      s.print(' ');
    }

    void print(Stream& s, const long& v) const
    {
      s.print(v);
      s.print(' ');
    }

    void print(Stream& s, float& v, int len) const
    {
      s.print(v, len);
      s.print(' ');
    }

    void print(Stream& s, const VectorFloat& v) const
    {
      print(s, v.x);
      print(s, v.y);
      print(s, v.z);
    }

    void print(Stream& s, const VectorInt16& v) const
    {
      print(s, v.x);
      print(s, v.y);
      print(s, v.z);
    }

    void print(Stream& s, const Quaternion& v) const
    {
      print(s, v.w);
      print(s, v.x);
      print(s, v.y);
      print(s, v.z);
    }

    void println(Stream& s) const
    {
      s.println();
    }

    Model& _model;
    mutable int _value;
    mutable bool _up;
};

}

#endif
