# ESP-FC Flight Controller
The mini, DIY, ~$5 cost, ESP8266 based, high performance flight controller.

## Features
* Frames (Quad X)
* Betaflight contiguration tool compatible
* Receiver protocol (8 channel PPM)
* SBUS and CRSF Serial Rx protocols on ESP32 and RP2040
* ESC protocols (PWM, Oneshot125, Oneshot42, Multishot, Brushed, Dshot150, Dshot300, Dshot600)
* Configurable Gyro Filters (LPF, Notch, dTerm)
* Blackbox recording (OpenLog serial)
* In flight PID Tuning
* Flight modes (ACRO, ANGLE, AIRMODE, ARM)
* Up to 2kHz gyro/loop in acro, 1kHz with accelerometer (level)
* MSP protocol interface
* Cli interface
* Resorce mapping
* Buzzer
* Lipo voltage monitor
* Failsafe

## Requirements
Hardware:
* Wemos D1 Mini board (D1 Mini Lite too, ESP-12 for experienced users)
* MPU6050 I2C gyro (GY-88, GY-91, GY-521 or similar)
* PDB with 5V BEC
* Buzzer and some electronic components (optionally).

Software:
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.9)
* [CH340 usb-serial converter driver](https://sparks.gogo.co.nz/ch340.html)
* [Espressif Flash Download Tools](https://www.espressif.com/en/support/download/other-tools)

## Flashing
**Windows:** download and unzip [Espressif Flash Download Tools](https://www.espressif.com/en/support/download/other-tools)

![ESP-FC Flashing](https://github.com/rtlopez/esp-fc/blob/master/docs/images/espfc_flashing.png?raw=true)

## Configuration
After flashing you need to configure few things first:
 1. Configure pinout according to your wiring, especially pin functions, you can find more information in [CLI Reference](https://github.com/rtlopez/esp-fc/blob/master/docs/cli.md)
 2. Connect to [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) and setup to your preferences,
 3. Test motors without propellers
 4. Have fun ;)

The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported.

## Wiring diagram
![ESP-FC Wemos D1 mini wiring diagram](https://github.com/rtlopez/esp-fc/blob/master/docs/images/espfc_wemos_d1_mini_wiring.png?raw=true)

## Supported protocols

| Protocol        | ESP8266 | ESP32 | RP2040 |
|----------------:|--------:|------:|-------:|
| UART            | Yes     |   Yes |    Yes |
| I2C             | Yes     |   Yes |    Yes |
| SPI             | -       |   Yes |    Yes |
| MSP             | Yes     |   Yes |    Yes |
| CLI             | Yes     |   Yes |    Yes |
| PPM (IN)        | Yes     |   Yes |    Yes |
| SBUS            | -       |   Yes |    Yes |
| IBUS            | -       |     - |      - |
| CRSF (ELRS)     | -       |   Yes |    Yes |
| BLACKBOX        | Yes     |   Yes |    Yes |
| PWM (OUT)       | Yes     |   Yes |    Yes |
| ONESHOT125      | Yes     |   Yes |    Yes |
| ONESHOT42       | -       |   Yes |    Yes |
| MULTISHOT       | -       |   Yes |    Yes |
| DSHOT150        | Yes     |   Yes |    Yes |
| DSHOT300        | Yes     |   Yes |    Yes |
| DSHOT600        | -       |   Yes |    Yes |

## Supported devices

| Device          | ESP8266 | ESP32 | RP2040 |
|----------------:|--------:|------:|-------:|
| MPU6050/I2C     | Yes     |   Yes |    Yes |
| MPU6000/SPI     | -       |     ? |      ? |
| MPU6500/I2C     | ?       |     ? |      ? |
| MPU6500/SPI     | -       |     ? |      ? |
| MPU9250/I2C     | ?       |     ? |      ? |
| MPU9250/SPI     | -       |   Yes |    Yes |
| BMP280/I2C      | Yes     |   Yes |    Yes |
| BMP280/SPI      | -       |   Yes |    Yes |
| HMC5883/I2C     | Yes     |   Yes |    Yes |
| HMC5883/SPI     | -       |     ? |      ? |
| AK8963/I2C      | -       |   Yes |    Yes |
| ICM20602/I2C    | ?       |     ? |      ? |
| ICM20602/SPI    | -       |   Yes |    Yes |

? - not tested, but should work

## Issues
You can report issues using Github [tracker](https://github.com/rtlopez/esp-fc/issues)

## Development
* Visual Studio Code
* [PlatformIO](https://platformio.org/install/ide?install=vscode)
* Git

## Todo
* Update documentation
* Balancing robot
* Serial Rx (IBUS)
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
