#ifndef _ESPFC_STEPPER_H_
#define _ESPFC_STEPPER_H_

#include <Arduino.h>
#include "EspTimer.h"
#include "AccelStepper.h"

void initPin(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

static void IRAM_ATTR forward()
{
  digitalWrite(2, 1);
  digitalWrite(25, 1);
  digitalWrite(25, 0);
  D("F");
}

static void IRAM_ATTR backward()
{
  digitalWrite(2, 0);
  digitalWrite(25, 1);
  digitalWrite(25, 0);
  D("B");
}

static AccelStepper accelStepper(forward, backward);
/*
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onTimer()
{
  digitalWrite(0, HIGH);
  portENTER_CRITICAL_ISR(&timerMux);
  digitalWrite(0, LOW);

  digitalWrite(0, HIGH);
  //accelStepper.run();
  digitalWrite(0, LOW); 

  digitalWrite(0, HIGH);
  portEXIT_CRITICAL_ISR(&timerMux);
  digitalWrite(0, LOW);
}*/

static uint32_t counter = 0;

namespace Espfc {

class Stepper
{
  public:
    void begin()
    {
      initPin(0);
      initPin(2);
      initPin(25);
      initPin(26);
      initPin(27);

      D("STEPPER");
      
      //_timer.begin(ESP_TIMER0, onTimer);
      //_timer.write(20000);

      accelStepper.setAcceleration(50);
      accelStepper.setMaxSpeed(500);
      //accelStepper.setSpeed();
      
      D(100);
    }

    void update()
    {
      static uint32_t last = 0;
      uint32_t now = micros();
      if(now - last >= 100)
      {
        last = now;
        counter++;

        digitalWrite(0, HIGH);
        accelStepper.runSpeed();
        digitalWrite(0, LOW);       

        digitalWrite(0, HIGH);
        switch(counter)
        {
          case 20000:
            _speed = 500;
            //accelStepper.setSpeed(20);
            //accelStepper.move(20);
            //accelStepper.moveTo(100);
            D(100);
            break;
          case 40000:
            _speed = 0;
            //accelStepper.setSpeed(0);
            //accelStepper.moveTo(0);
            D(0);
            break;
          case 60000:
            _speed = -500;
            //accelStepper.setSpeed(-20);
            //accelStepper.move(-20);
            //accelStepper.moveTo(-100);
            D(-100);
            break;
          case 80000:
            _speed = 0;
            //accelStepper.setSpeed(0);
            //accelStepper.moveTo(0);
            D(0);
            counter = 0;
            break;
        }
        _current  += (_speed - _current) * 0.001;
        accelStepper.setSpeed(_current);
        digitalWrite(0, LOW);       
      }
    }

  private:
    EspTimerImpl _timer;
    float _speed;
    float _current;
};

}

#endif