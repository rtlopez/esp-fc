#include <Wire.h>
#include <SPI.h>
#include <Ticker.h>
#include <Arduino.h>
#include <Espfc.h>

Espfc::Espfc espfc;
Ticker ticker;

static void blink_led()
{
  int state = digitalRead(LED_BUILTIN);
  digitalWrite(LED_BUILTIN, !state);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  ticker.attach(0.05, blink_led);

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println();

  espfc.begin();

  ticker.attach(0.2, blink_led);
}

void loop()
{
  espfc.update();
  yield();
}

int main()
{
  setup();
  while(true) loop();
  return 0;
}
