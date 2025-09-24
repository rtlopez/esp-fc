# ESP-FC Flight Controller

The mini, DIY, Low cost, ESP32 based, high performance flight controller for hobbyists.

# Features

* Espressif targets (ESP32, ESP32-S3)
* ESC protocols (PWM, Oneshot125/42, Multishot, Brushed, Dshot150/300/600 bidirectional)
* PPM, SBUS, IBUS and CRSF Receivers
* Builtin ESP-NOW receiver and WiFi configuration [read more...](/docs/wireless.md)
* SPI and I2C gyro modules support (MPU6050, MPU9250, ICM20602, BMI160)
* Flight modes (ACRO, ANGLE, AIRMODE)
* Frames (Quad X)
* Betaflight configuration tool compatible (v10.10)
* Configurable Gyro Filters (LPF, Dynamic Notches, dTerm, RPM)
* Blackbox recording (OpenLog/OpenLager/Flash)
* Up to 4kHz gyro/loop on ESP32 with SPI gyro
* MSP and CLI protocol interface
* Resorce/Pin mapping
* In flight PID Tuning
* Buzzer, Led and voltage monitor
* Failsafe mode

# Documentation

In this repository you can find firmware code that allows you to build your own flight controller. For convenience it mimics Betaflight 4.2 compability, so that it can be configured using [betaflight-configurator](https://github.com/betaflight/betaflight-configurator). Also [online blackbox-log-viewer](https://blackbox.betaflight.com/) can be used to analyze blackbox logs. In most aspects it is similar to Betaflight, so that many configuration and tuning advices are helpfull here too. But you must be aware, that this software is not the same as Betaflight, there are some limitations and differences in functionality and performance.

> [!IMPORTANT]
> Before you begin, **read the following documentation carefully first!**.

 * [Setup Guide](/docs/setup.md)
 * [Wiring](/docs/wiring.md)
 * [CLI Commands](/docs/cli.md)
 * [WIFI and ESP-NOW Receiver](/docs/wireless.md)

Join our **[Discord Channel](https://discord.gg/jhyPPM5UEH)** to get help

# Quick Start

## Requirements

Hardware:
* ESP32 or ESP32-S3 board
* MPU9250 SPI or MPU6050 I2C gyro (GY-88, GY-91, GY-521 or similar)
* PDB with 5V BEC
* Buzzer and some electronic components (optional).

Software:
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.10)
* [CH340 usb-serial converter driver](https://sparks.gogo.co.nz/ch340.html)

## Flashing

1. Download and unpack selected firmware from [Releases Page](https://github.com/rtlopez/esp-fc/releases)
2. Visit [ESP Tool Website](https://espressif.github.io/esptool-js/)
3. Click "Connect" and choose device port in dialog
4. Add firmware file and set Flash Address to `0x00`
5. Click "Program"
6. After success power cycle board

![ESP-FC Flashing](/docs/images/esptool-js-flash-connect.png)

## Setup

After flashing you need to configure few things first:

 1. Configure pinout according to your wiring, especially pin functions, you can find more information in [CLI Reference](/docs/cli.md)
 2. Connect to [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) and setup to your preferences,
 3. Test motors without propellers
 4. Have fun ;)

> [!NOTE]
> Not all functions displayed in configurator are avalable in firmware. The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported. It usually rolls back to previous value after save. It is strongly recommended to follow [setup guide](/docs/setup.md).

## Wiring diagrams

[![ESP-FC example wiring diagrams](/docs/images/espfc_wiring_combined.png)](/docs/wiring.md)

## Supported Modules

 * **ESP32** - recommended
 * **ESP32-S3** - recommended
 * **ESP32-S2** - experimantal
 * **ESP32-C3** - experimantal, lack of performance, no FPU
 * **RP2350** - experimantal, partially works
 * **RP2040** - experimantal, lack of performance, no FPU
 * **ESP8266** - obsolete, no longer developed

## Supported Sensors and Protocols

 * Gyro: MPU6050, MPU6000, MPU6500, MPU9250, ICM20602, BMI160
 * Barometers: BMP180, BMP280, SPL06
 * Magnetometers: HMC5883, QMC5883, AK8963
 * Receivers: PPM, SBUS, IBUS, CRSF/ELRS
 * Esc Protocols: PWM, BRUSHED, ONESHOT125, ONESHOT42, MULTISHOT, DSHOT150, DSHOT300, DSHOT600
 * Other protocols: MSP, CLI, BLACKBOX, ESPNOW

## Issues

You can report issues using Github [tracker](https://github.com/rtlopez/esp-fc/issues). 
You can also join our [Discord Channel](https://discord.gg/jhyPPM5UEH)

## Development

* Visual Studio Code
* [PlatformIO](https://platformio.org/install/ide?install=vscode) extension
* Git

## Todo

* Altitude Hold
* GPS Navigation
* MS5611 barometer
* Balancing robot controller

## Licence

This project is distributed under MIT Licence. Bear in mind that:

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Donations
If you like this project and you want it to be still developed, you can support me a little.

* BTC: 1Lopez7yPtbyjfLGe892JfheDFJMMt43tW
* LTC: LV3G3sJxz9AYpDMYUp8e1LCmerFYxVY3ak

## Extra Steps

In order to use the UART Base cli command issue use the following command
* pio device monitor --baud 115200

In order to use the UART cli as betaflight cli after the above command to enter UART cli type in `exit` and press enter now you should have entered beatflight cli

Also to compile_upload
* pio run -e esp32 -t upload

In my case have changed the sda pin to 14 as the default was a staffing pin , SCL is pin 22

* set pin_i2c_sda 14
* save


Also for CSRF set the 

* set serial_2_0_function_mask 64    # Enable RX_SERIAL on Serial2
* set serialrx_provider 6           # Set to CRSF (6 = CRSF)
* save

For the servo mixer here are the value to set via the betaflight cli

    # smix script for singlecopter
    mixer custom

    mmix load airplane    # Motor1 as ESC output

    # smix
    smix reset
    smix 0 3 0  100 0 0 100 0
    smix 1 2 0 -100 0 0 100 0
    smix 2 4 1  100 0 0 100 0
    smix 3 5 1 -100 0 0 100 0
    smix 4 3 2 50 0 0 100 0
    smix 5 2 2 50 0 0 100 0
    smix 6 4 2 50 0 0 100 0
    smix 7 5 2 50 0 0 100 0

Now for the servo update value use a lower refresh rate

    set servo_lowpass_hz = 20
    set servo_pwm_rate = 250
    save