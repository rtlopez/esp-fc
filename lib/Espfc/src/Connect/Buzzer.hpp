#pragma once

#include "Model.h"

namespace Espfc {

namespace Connect {

enum BuzzerPlayStatus
{
  BUZZER_STATUS_IDLE,
  BUZZER_STATUS_TEST,
  BUZZER_STATUS_ON,
  BUZZER_STATUS_OFF
};

class Buzzer
{
public:
  Buzzer(Model& model);
  int begin();
  int update();

private:
  void _play(bool v, int time, BuzzerPlayStatus s);
  void _write(bool v);
  void _delay(int time);

  static const uint8_t** schemes();

  Model& _model;

  BuzzerPlayStatus _status;
  uint32_t _wait;
  const uint8_t * _scheme;
  BuzzerEvent _e;
};

}

}
