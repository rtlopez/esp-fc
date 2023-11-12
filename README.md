# ESP-FC Flight Controller
The mini, DIY, ~$5 cost, ESP8266/ESP32 based, high performance flight controller.

## Features
* Frames (Quad X)
* Betaflight contiguration tool compatible (v10.8 or v10.9)
* Receiver protocol (8 channel PPM)
* SBUS and CRSF Serial Rx protocols on ESP32 and RP2040
* ESC protocols (PWM, Oneshot125, Brushed, Dshot150, Dshot300, Dshot600)
* Configurable Gyro Filters (LPF, Notch, dTerm)
* Blackbox recording (OpenLog/Opelager serial)
* In flight PID Tuning
* Flight modes (ACRO, ANGLE, AIRMODE, ARM)
* Up to 8kHz gyro/loop on ESP32 with SPI gyro
* MSP protocol interface
* CLI Interface
* Resorce/Pin mapping
* Buzzer
* Lipo voltage monitor
* Failsafe

## Requirements
Hardware:
* ESP8266 Wemos D1 Mini or ESP32 mini board
* MPU6050 I2C gyro (GY-88, GY-91, GY-521 or similar) or MPU9250 SPI on ESP32
* PDB with 5V BEC
* Buzzer and some electronic components (optional).

Software:
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.8 or v10.9)
* [CH340 usb-serial converter driver](https://sparks.gogo.co.nz/ch340.html)

## Flashing
1. Download and unzip selected firmware from [Releases Page](https://github.com/rtlopez/esp-fc/releases)
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
> Not all functions displayed in configurator are avalable in firmware. The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported. It usually rolls back to previous value after save.

Here are more details about [how to setup](/docs/setup.md).

## Wiring diagrams

[![ESP-FC example wiring diagrams](/docs/images/espfc_wiring_combined.png)](/docs/wiring.md)


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
| MPU6500/I2C     | ?       |   Yes |      ? |
| MPU6500/SPI     | -       |   Yes |    Yes |
| MPU9250/I2C     | Yes     |   Yes |    Yes |
| MPU9250/SPI     | -       |   Yes |    Yes |
| BMP280/I2C      | Yes     |   Yes |    Yes |
| BMP280/SPI      | -       |   Yes |    Yes |
| HMC5883/I2C     | Yes     |   Yes |    Yes |
| HMC5883/SPI     | -       |     ? |      ? |
| AK8963/I2C      | -       |   Yes |    Yes |
| ICM20602/I2C    | ?       |     ? |      ? |
| ICM20602/SPI    | -       |   Yes |    Yes |
| BMI160/I2C      | ?       |   Yes |      ? |
| BMI160/SPI      | -       |   Yes |    Yes |

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
