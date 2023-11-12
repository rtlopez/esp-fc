# Development

## Compile code

```
pio run -e lolin32
```

## Merge image

This step is only required to prepare single image to upload through User Interface, like Web UI or Firmware flasher for ESP32 based chips.
```
python3 ~/.platformio/packages/tool-esptoolpy/esptool.py --chip ESP32 merge_bin -o .pio/build/lolin32/firmware_full.bin --target-offset 0x1000 --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 .pio/build/lolin32/bootloader.bin 0x8000 .pio/build/lolin32/partitions.bin 0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin 0x10000 .pio/build/lolin32/firmware.bin
```

## Flash device

```
pio run -e lolin32 -t upload
```

For ESP32 chip it expands to:
```
python3 ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 --port "/dev/ttyUSB0" --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 .pio/build/lolin32/bootloader.bin 0x8000 .pio/build/lolin32/partitions.bin 0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin 0x10000 .pio/build/lolin32/firmware.bin
```

To flash merged file 

```
python3 ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 --port "/dev/ttyUSB0" --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 .pio/build/lolin32/firmware_full.bin
```

## Run unit tests

```
pio test -e native
```

## Docker

If you don't want to install PlatformIO

```
docker-compose run pio <pio command>
```

examples

```
docker-compose run pio pio run -e lolin32
docker-compose run pio pio test -e native
```
Keep in mind, that to be able to flash device, additional effort is required as by default docker is isolated enviroment devices are't accessible.

## VSCode

This project is based on [platformio](https://platformio.org/), it is recommended to install it as VSCode IDE extension.

1. if you don't have VSCode yet? visit https://code.visualstudio.com/download
2. then install https://platformio.org/install/ide?install=vscode
