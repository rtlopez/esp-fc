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
  //Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println();

  espfc.model().config.gyroFifo = true;
  espfc.model().config.gyroDlpf = Espfc::DLPF_188;
  espfc.model().config.gyroFsr  = Espfc::GYRO_FS_2000;
  espfc.model().config.accelFsr = Espfc::ACCEL_FS_8;
  espfc.model().config.gyroSampleRate = 50;
  espfc.model().config.magSampleRate  = 50;
  espfc.model().config.telemetry = true;
  espfc.model().config.telemetryInterval = 50;

  espfc.begin();

  ticker.attach(0.2, blink_led);
}

void loop()
{
  espfc.update();
  int now = millis();
  static int prev = 0;
  if(false && now - prev > 500)
  {
    Serial.print("echo ");
    Serial.print(espfc.model().config.telemetry); Serial.print(' ');
    Serial.print(espfc.model().state.timestamp); Serial.print(' ');
    Serial.println();
    prev = now;
  }
}

int main()
{
  setup();
  while(true) loop();
  return 0;
}
