#include <Wire.h>
#include <SPI.h>
#include "Arduino.h"

#include <Espfc.h>

Espfc::Espfc espfc;

void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  espfc.begin();
}

void loop()
{
  espfc.update();
}

int main()
{
  setup();
  while(true) loop();
  return 0;
}
