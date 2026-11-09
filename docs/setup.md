# Setup

This page describe steps absolutly required to proceed before first flight.

## Connecting to Betaflight Configurator

Open [Betaflight configurator](https://app.betaflight.com/) in the browser. It must be Chromium based browser (Chrome, Edge, Brave, etc.).

The **Connect** button with dropdown allows you to select your device.

![Connect](/docs/images/bfc/bfc2_connect.png)

At first time you may need to click `--- I can't find my device ---` to see all available serial devices. If you still can't see your device, click the gear icon in the bottom left corner and make sure `Show all serial devices` is enabled in the settings.

![Options](/docs/images/bfc/bfc2_options.png)

> [!NOTE]
> On Linux systems you might need to add your user to the `dialout` group to access the serial port: 
> ```
> sudo usermod -aG dialout $USER
> ```
> then log out and log in again.

Then select your device from list and click connect.

> [!NOTE]
> Not all functions displayed in configurator are avalable in firmware. The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported. It usually rolls back to previous value after save.

## Configure wiring

Go to the `CLI` tab and type `get pin`. This command will show what pins are associated to specified functions. If you wiring is diferrent, you can make some adjustments here.

![CLI Pins](/docs/images/bfc/bfc2_cli_pin.png)

For more details, see [Pin Functions CLI Reference](/docs/cli.md#pin-functions)

## Gyro calibration

Go to `Sensors` tab, Ensure that model in preview moves exactly in the same way as yours. If it is not the case, go to the `Sensors` tab, and configure `Board Alignment`

![Sensor Alignment](/docs/images/bfc/bfc2_sensors.png)

After adjustment make sure that your model is in horizontal steady state position. Then click **"Calibrate"** button in Accelerometer section and wait few seconds. Do not move the model during calibration process.

> [!CAUTION]
> Board Alignment is crucial for proper calibration and flight performance. Incorrect alignment will lead to uncontrolled spinning.

## System configuration

In `Configuration` tab set `Pid loop frequency`. Recomended values are 1kHz to 2kHz. On I2C gyro setups 1kHz is maximum recommended.

> [!NOTE]
> Keep eye on CPU Load displayed in bottom, it shoudn't exceed 50%.

## Receiver setup

It is recommended to use Serial based receiver (SBUS,IBUS,CRSF). You need to allocate UART port for it. You can do it in `Ports` tab, by ticking switch in `Serial Rx` column.

Then go to the `Receiver` tab, and select `Receiver mode` and `Serial Receiver Provider`.

To use ESP-NOW receiver, choose "SPI Rx (e.g. built-in Rx)" receiver mode in Receiver tab. You need compatible transmitter module. Read more about [ESP-FC Wireless Functions](/docs/wireless.md)

> [!NOTE]
> ESP-NOW is mostly for micro drones and not recommended for bigger setups due to limited range and reliability.

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

> [!WARNING]
> It is not recomended to use `PWM` for multirotors. Setups that use this protocol are hard to tune.

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

If you need more, choose `Serial Port` and serial device like [OpenLager](https://github.com/d-ronin/openlager) or [OpenLog](https://github.com/sparkfun/OpenLog). To configure it
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

`AirMode` is only available in modes tab. If you want to enable it permanently, use the same control channel as ARM.

### Failsafe tab

Only "Drop" Stage 2 procedule is possible

### GPS tab

You can change only `GPS for navigation and telemetry` and `Set Home point once` options and view GPS status. All other options are configurable via CLI tab only.

CLI commands are described in [GPS Setup](./gps.md) page.

### Presets tab

Presets aren't supported, do not try to apply any of them.

### PID Tuning tab

1. There is only one pid profile one rate profile and one battery profile
2. Feed-forward settings have no effect
3. Iterm-rotation has no effect
4. Dynamic damping has no effect
5. Throttle boost has no effect
6. Miscellaneous settings have no effect
7. Anti gravity settings have no effect
8. Dynamic Notch works from 1k pid loop

> [!NOTE]
> Besides that most of Betaflight principles can be applied here according to PID and Filter tuning. But keep in mind, very aggresive tunnig tips aren't recommended to apply and might lead to diferrent results.

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
