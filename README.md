# ESP-FC Flight Controller
The mini, DIY, ~$5 cost, ESP8266 based, high performance flight controller similar to Betaflight

## Features
* Frames (Quad X)
* Betaflight contiguration tool compatible
* Receiver protocol (8 channel PPM)
* ESC protocols (PWM, Oneshot125, Oneshot42, Multishot, Brushed)
* Configurable Gyro Filters (LPF, Notch, dTerm)
* Blackbox serial recording (OpenLog)
* In flight PID Tuning
* Flight modes (ACRO, ANGLE, AIRMODE, ARM)
* Up to 2kHz gyro/loop in acro, 1kHz with accelerometer
* MSP protocol interface
* Cli interface
* Resorce mapping
* Buzzer
* Lipo voltage monitor
* Failsafe

## Requirements
Hardware:
* Wemos D1 Mini board (D1 Mini Lite too, ESP-12 for experienced users)
* MPU6050 I2C gyro (GY-88, GY-521 or similar)

Software:
* PlatformIo Arduino Espressif8266 v2.4 SDK
* Atom IDE
* CH340 usb-serial converter driver

## Optional
* Buzzer and some electronic components.

## Flashing
The best way to flash your device is to install Atom IDE with platformio extension.
* https://atom.io/
* https://platformio.org/
* http://docs.platformio.org/en/latest/ide/atom.html#installation
* http://docs.platformio.org/en/latest/ide/atom.html#building-uploading-targets

To be able to build, you need to download or clone repository, and then open project in PlatformIo.
To build binary file press F7 and pick option 'PIO Build(d1_mini).
To upload, press F7 and choose 'PIO Upload(d1_mini)'

## Wiring diagram
![ESP-FC Wemos D1 mini wiring diagram](https://github.com/rtlopez/esp-fc/blob/master/docs/images/espfc_wemos_d1_mini_wiring.png?raw=true)

## Issues
You can Report issues using Github tracker

## Todo
* Update documentation
* Mixers: Quad+, Bicopter, Tricopter, Helicopter, Custom and more
* Balancing robot
* Serial Rx (SBUS, IBUS)
* Baro (MS5611, BMP280)
* Magnetometer (HMC5883, AK8963)
* GPS navigation
* ESP32 board
* ESP32 Dshot
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
