# ESP-FC Wiring Examples

## ESP32

ESP32 mcu allows to remap pins, so the wiring is not final and can remap intputs and outputs to your needs. To change pin function go to the CLI and use `get pin` command to check current assignment. For example, to set first output to pin 1 use command `set pin_output_0 1`.

### ESP32 SPI MPU-9250 gyro

![ESP-FC ESP32 SPI Wiring](./images/esp-fc-esp32_spi_wiring.png)

**SPI pins**

| PIN | Name | Module Names    |
|----:|------|-----------------|
| 18  | SCK  | SCK / SCL       |
| 19  | MISO | SDA / SDI       |
| 23  | MOSI | SAO / SDO / ADO |

### ESP32 I2C MPU-6050 gyro

![ESP-FC ESP32 I2C Wiring](./images/esp-fc-esp32_i2c_wiring.png)

# ESP8266 I2C MPU-6050 gyro

ESP8266 has limited ability to remap pins, use `get pin` command to list available options.

![ESP-FC ESP8266 I2C Wiring](./images/espfc_wemos_d1_mini_wiring.png)
