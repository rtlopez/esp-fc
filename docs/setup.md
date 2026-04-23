# Setup

This page describe steps absolutly required to proceed before first flight.

## Connecting to Betaflight Configurator

Once you download and install [Betaflight configurator](https://github.com/betaflight/betaflight-configurator) you need to change few options first. Open configurator and go to **Options** tab, and then
1. Disable `Advanced CLI AutoComplete`
2. Enable `Show all serial devices`
3. Disable `Auto-Connect`

![Options](/docs/images/bfc/bfc_options.png)

Then select your device from list and click connect.

> [!NOTE]
> Not all functions displayed in configurator are avalable in firmware. The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported. It usually rolls back to previous value after save.

> [!NOTE]
> Leaving some tabs, especially CLI requires you to click "Disconnect" button twice, and then click "Connect" again.

## Configure wiring

Go to the `CLI` tab and type `get pin`. This command will show what pins are associated to specified functions. If you wiring is diferrent, you can make some adjustments here. For more details, see [Pin Functions CLI Reference](/docs/cli.md#pin-functions)

![CLI Pins](/docs/images/bfc/bfc_cli_pins.png)

## Gyro calibration

Go to `Setup tab`, make sure that your model is in horizontal steady state position. Then click **"Calibrate Accelerometer"** button and wait two seconds.

> [!CAUTION]
> Ensure that model in preview moves exactly in the same way as yours. If it is not the case, go to the `Configuration` tab, and configure `Board orientation` or `Sensor Alignment`

![Sensor Alignment](/docs/images/bfc/bfc_configuration.png)

## System configuration

In `Configuration` tab set `Pid loop frequency`. Recomended values are 1kHz to 2kHz. Keep eye on CPU Load displayed in bottom, it shoudn't exceed 50%.

## Receiver setup

If you want to use Serial based receiver (SBUS,IBUS,CRSF), you need to allocate UART port for it. You can do it in `Ports` tab, by ticking switch in `Serial Rx` column.

Then go to the `Receiver` tab, and select `Receiver mode` and `Serial Receiver Provider`.

To use ESP-NOW receiver, choose "SPI Rx (e.g. built-in Rx)" receiver mode in Receiver tab. You need compatible transmitter module. Read more about [ESP-FC Wireless Functions](/docs/wireless.md)

## Motor setup

In `Motors` tab you must configure

### Mixer

You can select mixer type here, this configuration depends on type of aircraft you are configuring. 

**Important!** If you build multirotor, you must ensure that

1. Specified motor number is connected to specified output and placed in specified position in your aircraft according to gyro orientation, presented on a picture.
2. Specified motor is rotating in correct direction according to image.

> [!CAUTION] 
> If these conditions aren't met, your quad will go crazy on the first start and may cause damage or even injury.

To verify that you can enable **test mode** and spin each motor selectively. To do that,
1. remove all propellers, 
2. connect batery,
3. click "I understand the risk...", 
4. move specified slider to spin motor.

If you are using any analog protocol (PWM, OneShot, Multishot), you also need to calibrate your ESCs here. To do that
1. click "I understand the risk...", 
2. move master sliders to highest value
3. connect battery
4. when escs plays calibration beeps, move master slider down to lowest value
6. escs should play confirmation beeps

> [!NOTE]
> Only Quad X supported at the moment. Motor direction wizard is not supported, you must configure your ESC separately. The easiest way to change direction is swapping motor wires.

### Motor direction is reversed

This option informs FC that your motors rotating in reverse order. This option desn't reverse motor direction. It has to be configured with ESC configurator or by swapping motor wires.

### ESC protocol

You can select here protocol according to that you ESC can handle. For multirotor it is recommended to use `DSHOT150` or `DSHOT300`. Older ESCs may not support digital protocols, in this case it is recommended to use `OneShot125` at least. `Brushed` protocol is for driving brushed motors through FET drivers.

It is not recomended to use `PWM` for multirotors.

### Motor PWM speed separated from PID speed

This option allows to separate PWM frequency. If your pid loop is set to 1k, but ESC can accept maximum 333Hz. PWM is limitted to 480Hz.

## Flight modes

Flight modes can be configured in `Modes` tab

### Arm

It is required to enable this mode to be able to fly. Armed quad cause that motors are spinning (unless Motor stop option is enabled).

### Angle

By default ACRO mode is engaged if none of flying mode is active. This mode enables self-leveling. In this case you control titlt angle of multirotor instead of rotationg rate. It affects only Roll and Pitch axes. Yaw axis behave the same way as in Acro mode.

### Air mode

Increases the range of control at extreme throttle positions. Recommended to enable, can be controlled by the same RC channel as Arm to get permanent behaviour. Activates, when throttle achive about 40% remains active until disarmed.

### Buzzer

Acitvates buzzer, for example to find lost model in high grass.

### Fail safe

Triggers Failsafe procedure.

## Blackbox

It is possible to collect flight data in two ways. Via `serial port` or with `onboard flash`

Onboard flash allows to store about 2.5MB of data. This is equivalent of 2-3 minutes of flight. It should be enough for tuning. 

If you need more, choose `Serial Port` and serial device like [D-ronin OpenLager](https://github.com/d-ronin/openlager) or [OpenLog](https://github.com/sparkfun/OpenLog). To configure it
1. In `Ports` select uart to generate stream and in Peripherals column select `Blackbox logging` on free port
2. Then in `Blackbox` tab select `Serial Port` or `Onboard flash` as `logging device`

> [!NOTE]
> Port speed from column `Configuration/MSP` is used, and the same speed must be used in logging device, _(this might be subject of change in a future versions)_.

Recommended settings

- To log with 1k rate, you need device that is capable to receive data with 500kbps.
- To log with 2k rate, you need 1Mbps

OpenLager can handle it easily. If you plan to use OpenLog, you might need to flash [blackbox-firmware](https://github.com/cleanflight/blackbox-firmware) to be able to handle more than 115.2kbps. Either 250kbps and 500kbps works well with modern SD cards up to 32GB size.

## Limitations

### Configuration tab

In Other features you can enable only `Dynamic Filter`, `GPS`, and `SoftSerial`

`AirMode` is only available in modes tab. If you want to enable it permanently, use the same control channel as ARM.

### Failsafe tab

Only "Drop" Stage 2 procedule is possible

### GPS tab

GPS configuration is currently CLI-only. Use the commands documented in the GPS Setup section above. The GPS tab in Betaflight Configurator shows GPS status but cannot modify GNSS constellation settings.

### Presets tab

Presets aren't supported, do not try to apply any of them.

### PID Tuning tab

1. There is only one pid profile and one rate profile
2. Feed-forward transition has no effect
3. Iterm-rotation has no effect
4. Dynamic damping has no effect
5. Throttle boost has no effect
6. Miscellaneous settings has no effect
7. Dynamic Notch works from 1k pid loop, but is not displayed in configurator if this speed is configured, to reconfigure switch pid loop to 2k.

Besides that most of Betaflight principles can be applied here according to PID and Filter tuning. But keep in mind, very aggresive tunnig tips aren't recommended to apply and might lead to diferrent results.

### Receiver

1. Not all protocols are implemented, currently only PPM, CRSF, SBUS, IBUS
2. Only CRSF telemetry and rssi_adc
3. RC deadband applies to RPY, no separate Yaw deadband

### Modes

Add range works only, no add link

### Adjustments

Not implemented, for replacement you can use [pid scaler CLI](/docs/cli.md#scaler-configuration)

### Servos

Not Implemented, for replacemnt you can use [output cli](/docs/cli.md#output-channel-config) to change output protocol and rate.

### Motors

No 3D features, only Quad-X Mixer

### Video transmitter

Not yet implemented, work in progess

### OSD

Not implemented, but MW_OSD should works via serial port and MSP protocol.

## GPS Setup

ESP-FC supports u-blox M8, M9, F9, and M10 GPS modules via UART. M10 modules provide enhanced accuracy with dual-band L1+L5 GNSS support.

### Hardware Connection

1. Connect GPS TX to ESP32 RX pin (e.g., UART2 RX)
2. Connect GPS RX to ESP32 TX pin (e.g., UART2 TX)
3. Connect VCC (3.3V or 5V depending on module) and GND

### Enable GPS Feature

In the `Configuration` tab, under `Other Features`, enable **GPS** or use CLI:

```
set feature_gps 1
save
```

### Configure Serial Port

In the `Ports` tab, Disable MSP function and enable GPS on the UART connected to your GPS module. Set baud rate to `115200` (recommended for M10) or `230400` for 25Hz update rate.

Alternatively, via CLI:

```
# For UART2 (common GPS port)
set serial_1 2 115200 115200
save
```

### Basic GPS Configuration

Go to CLI tab and configure basic GPS settings:

```
# Minimum satellites required for valid fix
set gps_min_sats 8

# Set home only once (0 = update home on each arm)
set gps_set_home_once 1

save
```

### M10 GNSS Configuration (Advanced)

M10 modules support multiple GNSS constellations and dual-band (L1+L5) for better accuracy. Configure via CLI:

#### Quick Preset Modes

```
# Mode 0: Auto (use individual constellation flags) - DEFAULT
set gps_gnss_mode 0

# Mode 1: GPS only (maximum compatibility, lowest power)
set gps_gnss_mode 1

# Mode 2: GPS + GLONASS (good for high latitudes)
set gps_gnss_mode 2

# Mode 3: GPS + Galileo (best accuracy in Europe)
set gps_gnss_mode 3

# Mode 4: GPS + BeiDou (optimized for Asia-Pacific)
set gps_gnss_mode 4

# Mode 5: All constellations (maximum satellites, best accuracy)
set gps_gnss_mode 5

save
reboot
```

> [!NOTE]
> Some modules are not capable to track all constellations at the same time. In this case module initialization may fail.

#### Dual-Band Configuration

```
# Enable L1+L5 dual-band on M9 (better multipath rejection)
set gps_enable_dual_band 1

# Disable dual-band (force L1 only for compatibility)
set gps_enable_dual_band 0

save
reboot
```

> [!NOTE]
> M8/M10 modules always use L1 single-band. The `gps_enable_dual_band` setting only affects M9 modules.
> If module do not support it, this option has no effect.

#### Individual Constellation Control

When `gps_gnss_mode 0`, you can enable/disable each constellation individually:

```
set gps_enable_gps 1         # GPS (USA)
set gps_enable_glonass 1     # GLONASS (Russia)
set gps_enable_galileo 1     # Galileo (EU)
set gps_enable_beidou 1      # BeiDou (China)
set gps_enable_qzss 1        # QZSS (Japan/Asia-Pacific)
set gps_enable_sbas 1        # SBAS/WAAS/EGNOS augmentation

save
reboot
```

### Configuration Examples

#### Maximum Accuracy (M10 Recommended)
```
set gps_gnss_mode 5              # All constellations
set gps_enable_dual_band 1       # L1+L5 dual-band
save
reboot
```
**Result:** GPS+GLONASS+Galileo+BeiDou+QZSS+SBAS with L1+L5  
**Power:** High

#### Balanced Performance (M10)
```
set gps_gnss_mode 3              # GPS + Galileo
set gps_enable_dual_band 1       # L1+L5 for multipath rejection
save
reboot
```
**Result:** GPS+Galileo with L1+L5  
**Power:** Medium

#### Battery Saver (Any Module)
```
set gps_gnss_mode 1              # GPS only
set gps_enable_dual_band 0       # L1 only
save
reboot
```
**Result:** GPS only, L1 band  
**Power:** Low

#### Urban/City Flying (M10)
```
set gps_gnss_mode 3              # GPS + Galileo
set gps_enable_dual_band 1       # L5 rejects building reflections
save
reboot
```
**Result:** GPS+Galileo with L1+L5  
**Power:** Medium

#### Asia-Pacific Optimized (M10)
```
set gps_gnss_mode 0              # Custom mode
set gps_enable_gps 1
set gps_enable_galileo 1
set gps_enable_beidou 1          # Strong in Asia
set gps_enable_qzss 1            # Regional augmentation
set gps_enable_glonass 0         # Disable to save power
set gps_enable_dual_band 1
save
reboot
```
**Result:** GPS+Galileo+BeiDou+QZSS with L1+L5  
**Power:** Medium

### Verification

After rebooting, check GPS status in CLI:

```
# View current GPS configuration
get gps

# Check GPS status (in another CLI session or via status command)
status
```

Look for initialization messages in the boot log:

```
GPS DET 115200               # Baud rate detected
GPS VER: 000A0000            # M10 module detected
GPS GNSS L1+L5 [GPS GLO GAL BDS QZSS SBAS]  # Configuration applied
GPS RATE 40/1                # 25Hz update rate (M10 at 230400 baud)
GPS UBX                      # UBX protocol enabled
GPS NAV5                     # Navigation mode: Airborne
```

### Troubleshooting

#### No GPS Fix
1. Check antenna has clear sky view (away from carbon fiber, metal)
2. Enable more constellations: `set gps_gnss_mode 5`
3. Wait 2-3 minutes for initial fix (TTFF - Time To First Fix)
4. Verify baud rate matches GPS module (115200 or 230400)

#### GPS Not Detected
1. Check wiring (TX/RX crossed between GPS and FC)
2. Verify serial port configuration in `Ports` tab
3. Try different baud rates (9600, 38400, 57600, 115200, 230400)
4. Check boot logs for `GPS ERROR` or `GPS DET` messages

#### Low Satellite Count
1. Move to open area with clear sky view
2. Enable all constellations: `set gps_gnss_mode 5`
3. Check antenna connection
4. Wait longer for satellites to be acquired

#### M10 Not Using Dual-Band
1. Verify module is actually M10 (check boot log: `GPS VER: 000A0000`)
2. Ensure `gps_enable_dual_band 1` is set
3. Check log for `GPS GNSS L1+L5` (not just `L1`)
4. Some M10 modules need firmware update for L5 support

#### Slow Fix Time

1. Enable more constellations: `set gps_gnss_mode 5`
2. Ensure SBAS is enabled: `set gps_enable_sbas 1`
3. Check for clear sky view (no buildings, trees blocking)
4. First fix always takes longer (cold start), subsequent fixes are faster
