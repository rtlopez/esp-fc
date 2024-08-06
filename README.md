# ESP-FC Flight Controller
The mini, DIY, Low cost, ESP32 based, high performance flight controller for hobbyists.

## Features
* Espressif targets (ESP32, ESP8266, ESP32-S3, ESP32-S2, ESP32-C3)
* ESC protocols (PWM, Oneshot125/42, Multishot, Brushed, Dshot150/300/600 bidirectional)
* PPM, SBUS and CRSF Receivers
* Builtin ESP-NOW receiver and WiFi configuration [read more...](/docs/wireless.md)
* SPI and I2C gyro modules support (MPU6050, MPU9250, ICM20602, BMI160)
* Flight modes (ACRO, ANGLE, AIRMODE)
* Frames (Quad X)
* Betaflight configuration tool compatible (v10.8-v10.10)
* Configurable Gyro Filters (LPF, Notch, dTerm, RPM)
* Blackbox recording (OpenLog/OpenLager/Flash)
* Up to 4kHz gyro/loop on ESP32 with SPI gyro
* MSP protocol interface
* CLI Interface
* Resorce/Pin mapping
* In flight PID Tuning
* Buzzer
* Lipo voltage monitor
* Failsafe mode

## Requirements
Hardware:
* ESP32 mini board or ESP8266 Wemos D1 mini or similar
* MPU9250 SPI or MPU6050 I2C gyro (GY-88, GY-91, GY-521 or similar)
* PDB with 5V BEC
* Buzzer and some electronic components (optional).

Software:
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.8 to v10.10)
* [CH340 usb-serial converter driver](https://sparks.gogo.co.nz/ch340.html)

## Flashing
1. Download and unzip selected firmware from [Releases Page](https://github.com/rtlopez/esp-fc/releases)
2. Visit [ESP Tool Website](https://espressif.github.io/esptool-js/)
3. Click "Connect" and choose device port in dialog
4. Add firmware file and set Flash Address to `0x00`
5. Click "Program"
6. After success power cycle board

Note: only ESP32 and ESP8266 can be flashed in this way.

![ESP-FC Flashing](/docs/images/esptool-js-flash-connect.png)

## Setup
After flashing you need to configure few things first:
 1. Configure pinout according to your wiring, especially pin functions, you can find more information in [CLI Reference](/docs/cli.md)
 2. Connect to [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) and setup to your preferences,
 3. Test motors without propellers
 4. Have fun ;)

> [!NOTE]
> Not all functions displayed in configurator are avalable in firmware. The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported. It usually rolls back to previous value after save.

Here are more details about [how to setup](/docs/setup.md).

## Wiring diagrams

[![ESP-FC example wiring diagrams](/docs/images/espfc_wiring_combined.png)](/docs/wiring.md)

## Supported chips

 - **ESP32** - recommended
 - **ESP32-S2** - experimantal
 - **ESP32-S3** - experimantal
 - **ESP32-C3** - experimantal, lack of performance, no FPU
 - **RP2040** - experimantal, lack of performance, no FPU
 - **ESP8266** - deprecated, may stop being developed

## Supported Interfaces

| Interface       | ESP8266 | ESP32 | RP2040 |
|----------------:|--------:|------:|-------:|
| UART            | Yes     |   Yes |    Yes |
| I2C             | Yes     |   Yes |    Yes |
| SPI             | -       |   Yes |    Yes |

## Supported Receiver Protocols

| Protocol        | ESP8266 | ESP32 | RP2040 |
|----------------:|--------:|------:|-------:|
| PPM             | Yes     |   Yes |    Yes |
| SBUS            | Yes     |   Yes |    Yes |
| CRSF (ELRS)     | Yes     |   Yes |    Yes |

## Supported Motor Protocols

| Protocol        | ESP8266 | ESP32 | RP2040 |
|----------------:|--------:|------:|-------:|
| PWM             | Yes     |   Yes |    Yes |
| BRUSHED         | Yes     |   Yes |    Yes |
| ONESHOT125      | Yes     |   Yes |    Yes |
| ONESHOT42       | -       |   Yes |    Yes |
| MULTISHOT       | -       |   Yes |    Yes |
| DSHOT150        | Yes     |   Yes |    Yes |
| DSHOT300        | Yes     |   Yes |    Yes |
| DSHOT600        | -       |   Yes |    Yes |

# Other Protocols

| Protocol        | ESP8266 | ESP32 | RP2040 |
|----------------:|--------:|------:|-------:|
| MSP             | Yes     |   Yes |    Yes |
| CLI             | Yes     |   Yes |    Yes |
| BLACKBOX        | Yes     |   Yes |    Yes |
| ESPNOW          | Yes     |   Yes |      - |

## Supported Gyro devices

| Device      | ESP8266 | ESP32 | RP2040 |
|------------:|--------:|------:|-------:|
| MPU6050     | Yes     |   Yes |    Yes |
| MPU6000     | -       |   Yes |    Yes |
| MPU6500     | Yes     |   Yes |    Yes |
| MPU9250     | Yes     |   Yes |    Yes |
| ICM20602    | Yes     |   Yes |    Yes |
| BMI160      | Yes     |   Yes |    Yes |

## Supported Baro devices

| Device      | ESP8266 | ESP32 | RP2040 |
|------------:|--------:|------:|-------:|
| BMP180      | Yes     |   Yes |    Yes |
| BMP280      | Yes     |   Yes |    Yes |
| SPL06       | Yes     |   Yes |    Yes |

## Supported Compass devices

| Device      | ESP8266 | ESP32 | RP2040 |
|------------:|--------:|------:|-------:|
| HMC5883     | Yes     |   Yes |    Yes |
| QMC5883     | Yes     |   Yes |    Yes |
| AK8963      | Yes     |   Yes |    Yes |

## Issues
You can report issues using Github [tracker](https://github.com/rtlopez/esp-fc/issues)
You can also join to our [Discord Channel](https://discord.gg/jhyPPM5UEH)

## Development
* Visual Studio Code
* [PlatformIO](https://platformio.org/install/ide?install=vscode)
* Git

## Todo
* Balancing robot
* Serial Rx (IBUS)
* ELRS telemetry
* ESP32-S2/S3/C3 targets
* Baro (MS5611)
* GPS navigation

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
