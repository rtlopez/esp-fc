# WEMOS D1 MINI connections

```
Pin | Function         | ESP-8266 Pin | ESPFC external device
----+------------------+--------------+----------------------------------------------
TX  | TXD0             | TXD, GPIO1   | (UART0 TX) | CLI | MSP | OSD | GPS 
RX  | RXD0             | RXD, GPIO3   | (UART0 RX) | CLI | MSP | OSD | GPS | SBUS
A0  | Analog In,  3.3V | A0           | Batery voltage or current sensor, analog only
D0  | IO               | GPIO16       | LED | BUZZER | PWM (output recommended, no interrupts)
D1  | IO, SCL          | GPIO5        | I2C Gyro, Compas, Pressure and other sensors
D2  | IO, SDA          | GPIO4        | I2C Gyro, Compas, Pressure and other sensors
D3  | IO, PU           | GPIO0        | PWM 4 (servo/motor)
D4  | IO, PU, LD, TXD1 | GPIO2        | Blackbox (UART1 TX)
D5  | IO, SCK          | GPIO14       | PWM 3 (servo/motor)
D6  | IO, MISO         | GPIO12       | PWM 2 (servo/motor)
D7  | IO, MOSI         | GPIO13       | PPM RX | PWM | LED | BUZZER
D8  | IO, SS, PD       | GPIO15       | PWM 1 (servo/motor)
G   | Ground           | GND          | Ground
5V  | 5V               | N/A          | 5V power
3V3 | 3.3V             | 3.3V         | 3.3V power - to I2C devices
RST | Reset            | RST          | Reset switch (optional)
```

## Legend:
```
   PU - 10k pull up
   PD - 10k pull down
 RXD0 - UART0 hardware RX
 TXD0 - UART0 hardware TX
 TXD1 - UART1 hardware TX only
   LD - builtin LED
```

## Notice:
```
 A0 - In ESP-8266 is 1.0V range, divider may be required
 D0 - No iterrupts
 D3, GPIO0 - if low at boot, then board enters to flashing mode
 D4, GPIO2 - must be high during flashing
```

## Wemos D1 mini board layout:
```
      +-----------+
   TX |           | RST
   RX |           | A0
   D1 |           | D0
   D2 |           | D5 ~>
<~ D3 |           | D6 ~>
<~ D4 |           | D7 ~>
  GND |           | D8 ~<
   5v |           | 3.3V
      +---|---|---+
         USB
```
