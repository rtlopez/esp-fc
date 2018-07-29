# WEMOS D1 MINI pinout
```
Pin | Function         | ESP-8266 Pin | ESPFC external device
----+------------------+--------------+----------------------------------------------
TX  | TXD0             | TXD, GPIO1   | (UART0 TX) | CLI | MSP | OSD | GPS
RX  | RXD0             | RXD, GPIO3   | (UART0 RX) | CLI | MSP | OSD | GPS | SBUS
A0  | Analog In,  3.3V | A0           | Batery voltage or current sensor, analog only
D0  | IO               | GPIO16       | LED | BUZZER | PWM (output recommended, no interrupts)
D1  | IO, SCL          | GPIO5        | I2C Gyro, Compas, Pressure and other sensors
D2  | IO, SDA          | GPIO4        | I2C Gyro, Compas, Pressure and other sensors
D3  | IO, PU           | GPIO0        | PWM 1 (servo/motor)
D4  | IO, PU, LD, TXD1 | GPIO2        | Blackbox (UART1 TX)
D5  | IO, SCK          | GPIO14       | PWM 2 (servo/motor)
D6  | IO, MISO         | GPIO12       | PWM 3 (servo/motor)
D7  | IO, MOSI         | GPIO13       | PPM RX | PWM
D8  | IO, SS, PD       | GPIO15       | PWM 4 (servo/motor)
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
 TXD1 - UART1 hardware TX
   LD - builtin LED
```

## Notice:
```
 A0 - In ESP-8266 is 1.0V range, Wemos has 220k:100k divider for 3.3V range, 22k:4k7 additional divider required for 4S Lipo
 D0 - No iterrupts (recommended output)
 D3, GPIO0 - if low at boot, then board enters to flashing mode (avoid using as input)
 D4, GPIO2 - must be high during flashing (avoid using as input)
 D8, GPIO15 - must be low during flashing or boot (avoid using as input)
```
https://github.com/esp8266/esp8266-wiki/wiki/Boot-Process#esp-boot-modes

## Wemos D1 mini board layout:
```
      /-----------\
   TX |           | RST
   RX |           | A0
   D1 |           | D0
   D2 |           | D5 ~>
<~ D3 |           | D6 ~>
<~ D4 |           | D7 ~<
  GND |           | D8 ~>
   5v |           | 3.3V
      +---|---|---+
           USB
```

# ESP32 pinout
```
Pin | Name | Function                                   | ESPFC external device
----+------+--------------------------------------------+----------------------------------------------
 0  |      | PU,ADC2_CH1,CLK_OUT1                       | >  M0
 1  | TXD  | U0TXD,CLK_OUT3                             |    TX0, PROG, MSP
 2  |      | ADC2_CH2,HSPIWP,HS2_DATA0,SD_DATA0         | >  M1
 3  | RXD  | U0RXD,CLK_OUT2                             |    RX0, PROG, MSP
 4  |      | ADC2_CH0,HSPIHD,HS2_DATA1,SD_DATA1         | $  BUZZER
 5  |      | VSPICS0,HS1_DATA6                          | >  SPI_CS0_GYRO, SPI0_SS
11  | CMD  | SD_CMD,SPICS0,HS1_CMD,U1RTS                | $  SPI_CS1_BARO
12  | TD1  | PD,ADC2_CH5,MTDI,HSPIQ,HS2_DATA2,SD_DATA2  | >  M5, SPI1_MISO 
13  | TCK  | ADC2_CH4,MTCK,HSPID,HS2_DATA3,SD_DATA3     |    M6, SPI1_MOSI
14  | TMS  | ADC2_CH6,MTMS,HSPICLK,HS2_CLK,SD_CLK       |    M7, SPI1_SCK
15  | TD0  | PU,ADC2_CH3,MTDO,HSPICS0,HS2_CMD,SD_CMD    | >  SPI_CS2_SDCARD, SPI1_SS
16  |      | HS1_DATA4,U2RXD                            |    RX2
17  |      | HS1_DATA5,U2TXD                            |    TX2
18  |      | VSPICLK,HS1_DATA7                          |    SPI0_SCK
19  |      | VSPIQ,U0CTS                                |    SPI0_MISO
21  |      | VSPIHD                                     |    I2C0_SCL
22  |      | VSPIWP,U0RTS                               |    I2C0_SDA
23  |      | VSPID,HS1_STROBE                           |    SPI0_MOSI
25  |      | ADC2_CH8,DAC1                              |    M2
26  |      | ADC2_CH9,DAC2                              |    M3
27  |      | ADC2_CH7                                   |    M4
32  |      | ADC1_CH4,XTAL                              |    RX1
33  |      | ADC1_CH5,XTAL                              |    TX1
34  |      | ADC1_CH6                                   | <$ ADC, INTR
35  |      | ADC1_CH7                                   | <  PPM, ADC, INTR
36  | SVP  | ADC1_CH0                                   | <  ADC_VOLTAGE
37  | -    | ADC1_CH1                                   | -  N/A
38  | -    | ADC1_CH2                                   | -  N/A
39  | SVN  | ADC1_CH3                                   | <  ADC_CURRENT

RESERVED
 6  | CLK  | SD_CLK,SPICLK,HS1_CLK,U1CTS                | (flash)
 7  | SD0  | SD_DATA0,SPIQ,HS1_DATA0,U2RTS              | (flash)
 8  | SD1  | SD_DATA1,SPID,HS1_DATA1,U2CTS              | (flash)
 9  | SD2  | SD_DATA2,SPIHD,HS1_DATA2,U1RXD             | (flash)
10  | SD3  | SD_DATA3,SPIWP,HS1_DATA3,U1TXD             | (flash)

NOT EXISTS
20  | -    | -                                          | -
24  | -    | -                                          | -
28  | -    | -                                          | -
29  | -    | -                                          | -
30  | -    | -                                          | -
31  | -    | -                                          | -
```

## Legend
```
 '>' recommended output only
 '<' input only
 '$' free
 '-' not available
```
GPIO `0, 2, 12, 15` recommended as output because of boot process impact
GPIO `34-39` are input-only by chip

- GPIO0 - Low/GND	ROM serial bootloader for esptool.py, High/VCC	Normal execution mode
- GPIO2 - GPIO2 must also be either left unconnected/floating, or driven Low, in order to enter the serial bootloader. In normal boot mode (GPIO0 high), GPIO2 is ignored.
- GPIO12 - If driven High, flash voltage (VDD_SDIO) is 1.8V not default 3.3V. Has internal pull-down, so unconnected = Low = 3.3V. May prevent flashing and/or booting if 3.3V flash is used and this pin is pulled high, causing the flash to brownout. See the ESP32 datasheet for more details.
- GPIO15 - If driven Low, silences boot messages printed by the ROM bootloader. Has an internal pull-up, so unconnected = High = normal output.
- GPIO5 - configures SDIO Slave in "Download Boot"

https://github.com/espressif/esptool/wiki/ESP32-Boot-Mode-Selection

## ESP32 mini board layout
```
                  /-------------\
 GND      RST     |             |  TXD (1)    GND
  NC      SVP(36) |             |  RXD (3)    IO(27)
 SVN(39)   IO(26) |             |   IO(22)    IO(25)
  IO(35)   IO(18) |             |   IO(21)    IO(32)
  IO(33)   IO(19) |             |   IO(17)   TD1(12)
  IO(34)   IO(23) |             |   IO(16)    IO(04)
 TMS(14)   IO(05) |             |  GND        IO(00)
  NC      3V3     |             |  VCC(5v)    IO(02)
 SD2(09)  TCK(13) |             |  TD0(15)   SD1(08)
 CMD(11)  SD3(10) |             |  SD0(07)   CLK(06)
                  +----|---|----+
                        USB
```