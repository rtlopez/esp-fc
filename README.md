# ESP-FC Flight Controller
The mini, DIY, ~$5 cost, ESP8266 based, high performance flight controller.

## Features
* Frames (Quad X)
* Betaflight contiguration tool compatible
* Receiver protocol (8 channel PPM)
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
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.7)
* [CH340 usb-serial converter driver](https://sparks.gogo.co.nz/ch340.html)
* [Espressif Flash Download Tools](https://www.espressif.com/en/support/download/other-tools)

## Flashing
**Windows:** download and unzip [Espressif Flash Download Tools](https://www.espressif.com/en/support/download/other-tools)

![ESP-FC Flashing](https://github.com/rtlopez/esp-fc/blob/master/docs/images/espfc_flashing.png?raw=true)

## Configuration
The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported.

## Wiring diagram
![ESP-FC Wemos D1 mini wiring diagram](https://github.com/rtlopez/esp-fc/blob/master/docs/images/espfc_wemos_d1_mini_wiring.png?raw=true)

## Issues
You can report issues using Github [tracker](https://github.com/rtlopez/esp-fc/issues)

## Development
* Visual Studio Code
* [PlatformIO](https://platformio.org/install/ide?install=vscode)
* Git

## Todo
* Update documentation
* Mixers: Quad+, Bicopter, Tricopter, Helicopter, Custom and more
* Balancing robot
* Serial Rx (SBUS, IBUS)
* Baro (MS5611, BMP280)
* Magnetometer (HMC5883, AK8963)
* GPS navigation
* ESP32 board
* ESP32 SPI Gyro (MPU9250)

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
