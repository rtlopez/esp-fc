# ESP-FC CLI Reference

## Help
```
help 
available commands:
 help
 dump
 get param
 set param value ...
 cal [gyro]
 defaults
 save
 reboot
 scaler
 mixer
 stats
 status
 devinfo
 version

```

## Information

### Version

```
version 
ESPF ESP8266 v0.0.0 0000000 Nov 13 2020 10:30:13 4.8.2 201103
```

### Status
```
status 
ESPF ESP8266 v0.0.0 0000000 Nov 13 2020 10:30:13 4.8.2 201103
STATUS: 
 cpu : 160 MHz
gyro : MPU6050/I2C
baro : NONE
 mag : NONE
 rx rate : 45
arming disabled : 33554560
```

### Statisticss

Ensure that total CPU usage doesn't exceed 60-70%
```
stats 
ESPF ESP8266 v0.0.0 0000000 Nov 13 2020 10:30:13 4.8.2 201103
    cpu freq: 160 MHz
   gyro rate: 1000 Hz
   loop rate: 1000 Hz
  mixer rate: 1000 Hz

 gyro_r: 119us, 11.9%
 gyro_f: 104us, 10.4%
  acc_r: 58us, 5.8%
  acc_f: 12us, 1.3%
  mag_r: 0us, 0.0%
  mag_f: 0us, 0.0%
 baro_p: 0us, 0.0%
  imu_p: 85us, 8.6%
  imu_c: 0us, 0.0%
   rx_r: 5us, 0.6%
   rx_f: 56us, 5.7%
     fs: 0us, 0.1%
   rx_a: 0us, 0.0%
  pid_o: 0us, 0.0%
  pid_i: 0us, 0.0%
  mixer: 50us, 5.1%
mixer_w: 123us, 12.3%
  bblog: 43us, 4.3%
    tlm: 0us, 0.0%
 serial: 4us, 0.5%
   wifi: 0us, 0.0%
  TOTAL: 666us, 66.7%
```

## Configuration

 - **defaults** - restore defaults
 - **save** - persist configuration after changes
 - **reboot** - reboot board

## Configuration parameters

 - **dump** - dump all paramters
 - **get parameter_name** - get paramter value, list matching patameters with current values
 - **set paramter value1 [value2] [value3]** - set parameter to new value (some parameters store multiple values, ex. input)

Many parameters have similar name and meaning like Betaflight.

### Configuration examples

Get single value
```
# get gyro_lpf_type
set gyro_lpf_type PT1
```

Get multiple values
```
# get output_ 
set mixer_output_limit 100
set output_motor_protocol DSHOT150
set output_motor_async 0
set output_motor_rate 480
set output_servo_rate 0
set output_min_command 1000
set output_min_throttle 1050
set output_max_throttle 2000
set output_dshot_idle 450
set output_0 M N 1000 1500 2000
set output_1 M N 1000 1500 2000
set output_2 M N 1000 1500 2000
set output_3 M N 1000 1500 2000
set pin_output_0 0
set pin_output_1 14
set pin_output_2 12
set pin_output_3 15
```

Setting single value
```
set pin_output_0 0
```
Not that you don't need to use `=` character for assignment

Setting multiple values
```
set output_0 S R
```
Note that you don't have to enter all values, you can ommit last values if they are not changed.

## All supported paramters

```
# dump

set features 536870913
set debug_mode GYRO_SCALED
set gyro_bus AUTO
set gyro_dev AUTO
set gyro_dlpf 256Hz
set gyro_sync 8
set gyro_align DEFAULT
set gyro_lpf_type PT1
set gyro_lpf_freq 100
set gyro_lpf2_type PT1
set gyro_lpf2_freq 188
set gyro_lpf3_type FIR2
set gyro_lpf3_freq 250
set gyro_notch1_freq 0
set gyro_notch1_cutoff 0
set gyro_notch2_freq 0
set gyro_notch2_cutoff 0
set gyro_dyn_lpf_min 150
set gyro_dyn_lpf_max 375
set gyro_offset_x -84
set gyro_offset_y -12
set gyro_offset_z -82
set accel_bus AUTO
set accel_dev AUTO
set accel_align DEFAULT
set accel_lpf_type BIQUAD
set accel_lpf_freq 10
set accel_offset_x 0
set accel_offset_y 0
set accel_offset_z 0
set mag_bus AUTO
set mag_dev NONE
set mag_align DEFAULT
set mag_filter_type BIQUAD
set mag_filter_lpf 10
set mag_offset_x 0
set mag_offset_y 0
set mag_offset_z 0
set mag_scale_x 1000
set mag_scale_y 1000
set mag_scale_z 1000
set baro_bus AUTO
set baro_dev NONE
set baro_lpf_type BIQUAD
set baro_lpf_freq 15
set fusion_mode MADGWICK
set fusion_gain 50
set input_rate_type BETAFLIGHT
set input_roll_rate 70
set input_roll_srate 80
set input_roll_expo 0
set input_roll_limit 1998
set input_pitch_rate 70
set input_pitch_srate 80
set input_pitch_expo 0
set input_pitch_limit 1998
set input_yaw_rate 120
set input_yaw_srate 50
set input_yaw_expo 0
set input_yaw_limit 1998
set input_deadband 3
set input_min 885
set input_mid 1500
set input_max 2115
set input_interpolation AUTO
set input_interpolation_interval 26
set input_rssi_channel 0
set input_0 0 1000 1500 2000 A 1500
set input_1 1 1000 1500 2000 A 1500
set input_2 3 1000 1500 2000 A 1500
set input_3 2 1000 1500 2000 A 1000
set input_4 4 1000 1500 2000 H 1500
set input_5 5 1000 1500 2000 H 1500
set input_6 6 1000 1500 2000 H 1500
set input_7 7 1000 1500 2000 H 1500
set input_8 8 1000 1500 2000 H 1500
set input_9 9 1000 1500 2000 H 1500
set input_10 10 1000 1500 2000 H 1500
set input_11 11 1000 1500 2000 H 1500
set input_12 12 1000 1500 2000 H 1500
set input_13 13 1000 1500 2000 H 1500
set input_14 14 1000 1500 2000 H 1500
set input_15 15 1000 1500 2000 H 1500
set serial_0 1 115200 0
set serial_1 128 115200 250000
set serial_soft_0 0 115200 0
set scaler_0 769 5 20 400
set scaler_1 770 6 20 400
set scaler_2 772 7 20 200
set mode_0 0 4 1300 2100 0 0
set mode_1 1 4 1700 2100 0 0
set mode_2 2 4 1300 2100 0 0
set mode_3 0 4 900 900 0 0
set mode_4 0 4 900 900 0 0
set mode_5 0 4 900 900 255 255
set mode_6 0 4 900 900 0 0
set mode_7 0 4 900 900 0 0
set pid_sync 1
set pid_roll_p 42
set pid_roll_i 85
set pid_roll_d 30
set pid_roll_f 90
set pid_pitch_p 46
set pid_pitch_i 90
set pid_pitch_d 32
set pid_pitch_f 95
set pid_yaw_p 45
set pid_yaw_i 90
set pid_yaw_d 0
set pid_yaw_f 90
set pid_level_p 55
set pid_level_i 0
set pid_level_d 0
set pid_level_angle_limit 55
set pid_level_rate_limit 300
set pid_level_lpf_type PT1
set pid_level_lpf_freq 90
set pid_yaw_lpf_type PT1
set pid_yaw_lpf_freq 90
set pid_dterm_lpf_type PT1
set pid_dterm_lpf_freq 90
set pid_dterm_lpf2_type PT1
set pid_dterm_lpf2_freq 113
set pid_dterm_notch_freq 0
set pid_dterm_notch_cutoff 0
set pid_dterm_dyn_lpf_min 53
set pid_dterm_dyn_lpf_max 128
set pid_dterm_weight 30
set pid_iterm_limit 30
set pid_iterm_zero 1
set pid_tpa_scale 10
set pid_tpa_breakpoint 1650
set mixer_sync 1
set mixer_type QUADX
set mixer_yaw_reverse 0
set mixer_throttle_limit_type CLIP
set mixer_throttle_limit_percent 60
set mixer_output_limit 100
set output_motor_protocol DSHOT150
set output_motor_async 0
set output_motor_rate 480
set output_servo_rate 0
set output_min_command 1000
set output_min_throttle 1050
set output_max_throttle 2000
set output_dshot_idle 450
set output_0 M N 1000 1500 2000
set output_1 M N 1000 1500 2000
set output_2 M N 1000 1500 2000
set output_3 M N 1000 1500 2000
set pin_input_rx 13
set pin_output_0 0
set pin_output_1 14
set pin_output_2 12
set pin_output_3 15
set pin_buzzer 16
set pin_i2c_scl 5
set pin_i2c_sda 4
set pin_input_adc 17
set pin_buzzer_invert 1
set i2c_speed 1000
set wifi_mode OFF
set wifi_ssid 
set wifi_pass 
set wifi_ssid_ap 
set wifi_pass_ap 
set wifi_tcp_port 1111
set mix_outputs 0
set mix_0 0 0 0
set mix_1 0 0 0
set mix_2 0 0 0
set mix_3 0 0 0
set mix_4 0 0 0
set mix_5 0 0 0
set mix_6 0 0 0
set mix_7 0 0 0
set mix_8 0 0 0
set mix_9 0 0 0
set mix_10 0 0 0
set mix_11 0 0 0
set mix_12 0 0 0
set mix_13 0 0 0
set mix_14 0 0 0
set mix_15 0 0 0
set mix_16 0 0 0
set mix_17 0 0 0
set mix_18 0 0 0
set mix_19 0 0 0
set mix_20 0 0 0
set mix_21 0 0 0
set mix_22 0 0 0
set mix_23 0 0 0
set mix_24 0 0 0
set mix_25 0 0 0
set mix_26 0 0 0
set mix_27 0 0 0
set mix_28 0 0 0
set mix_29 0 0 0
set mix_30 0 0 0
set mix_31 0 0 0
set mix_32 0 0 0
set mix_33 0 0 0
set mix_34 0 0 0
set mix_35 0 0 0
set mix_36 0 0 0
set mix_37 0 0 0
set mix_38 0 0 0
set mix_39 0 0 0
set mix_40 0 0 0
set mix_41 0 0 0
set mix_42 0 0 0
set mix_43 0 0 0
set mix_44 0 0 0
set mix_45 0 0 0
set mix_46 0 0 0
set mix_47 0 0 0
set mix_48 0 0 0
set mix_49 0 0 0
set mix_50 0 0 0
set mix_51 0 0 0
set mix_52 0 0 0
set mix_53 0 0 0
set mix_54 0 0 0
set mix_55 0 0 0
set mix_56 0 0 0
set mix_57 0 0 0
set mix_58 0 0 0
set mix_59 0 0 0
set mix_60 0 0 0
set mix_61 0 0 0
set mix_62 0 0 0
set mix_63 0 0 0
```

## Specific paramters reference

### Pin functions

Default pin assignments for ESP8266 is listed below
```
set pin_input_rx 13
set pin_output_0 0
set pin_output_1 14
set pin_output_2 12
set pin_output_3 15
set pin_buzzer 16
set pin_i2c_scl 5
set pin_i2c_sda 4
set pin_input_adc 17
set pin_buzzer_invert 1
```

You can swap two motor outputs (example 0 and 3) by running commands

```
set pin_output_0 15
set pin_output_3 0
save
reboot
```

**Note 1**: ESP8266 doesn't allow to remap UART pins, but can be used as outputs if device is inactive. But generally avoid using pins 0,1,2,3 as inputs as they are needed for programming. Pin 16 cannot be used as input, because doesn't handle interrupt, but can be used as output, ex. for buzzer.

**Note 2**: The numbers printed on the circuit boards do not always match the pin numbers of the GPIO. You need to consult it with board reference or at least with this [pin summary](https://github.com/rtlopez/esp-fc/blob/master/docs/connections.md) or [wiring diagram](https://github.com/rtlopez/esp-fc#wiring-diagram).

### Input channel config
```
set input_0 0 1000 1500 2000 A 1500
set input_{channel} {map} {min} {neutral} {max} {fs_mode} {fs_value}
```
 - `{channel}`: raw rc channel (0-index)
 - `{map}`: map raw channel to input channel (0-index)
 - `{min/neutral/max}`: same as `rxrange` to adjust radio endpoints, scale rx input range to 1000/1500/2000, enter your min/mid/max values transmitted by radio.
 - `{fs_mode}`: fail safe mode, can be A: auto, H: hold, S: set
 - `{fs_value}`: value to use when fsMode is S(set)

### Output channel config
```
set output_0 M N 1000 1500 2000
set output_{channel} {type} {inverse} {min} {neutral} {max}
```
 - `{channel}`: output channel (0-index)
 - `{type}`: output type, can be M: motor, S: servo
 - `{inverse}`: inverse output, can be set to N: normal, R: reversed
 - `{min/neutral/max}`: limit/adjust output

### Mode conditions
```
set mode_{index} {id} {channel} {min} {max} {logic_mode} {link_id}
set mode_0 0 4 1300 2100 0 0
```
 - `{index}`: mode activator index
 - `{id}`: mode Id
 - `{channel}`: observed input channel (0-index, must be greather than 3)
 - `{min/max}`: activation range, if both are set to 900, it means inactive
 - `{logic_mode}`: NOT IMPLEMENTED, used by configurator but ignored by firmare
 - `{link_id}` - NOT IMPLEMENTED, used by configurator but ignored by firmare

Mode IDs:
 - 0: arming
 - 1: angle
 - 2: air mode
 - 3: buzzer
 - 4: failsafe

### Serial port configuration
```
set serial_0 1 115200 0
set serial_{index} {function} {msp_baud} {blackbox_baud}
```
 - `{index}`: port number (0-index)
 - `{function}`: function id
 - `{msp_baud}`: msp baud rate
 - `{blackbox_baud}`: blackbox baud rate

Serial Functions:
 - 0: none
 - 1: msp
 - 128: blackbox

### Scaler configuration

It's diferrent version of inflight adjustments, allow to control few parameters with rc channel, ex. by potentiomenters.

```
set scaler_0 769 5 20 400
set scaler_{index} {dimensions} {channel} {min_scale} {max_scale}
```

 - `{index}`: scaler instance (0-3)
 - `{dimensions}`: combination of dimentions
 - `{channel}`: observed iput channel (0-index, must be greather than 3)
 - `{min_scale/max_scale}`: min and max scale in percent

Scaler Dimensions:
 - 1: PID P component of inner loop
 - 2: PID I component of inner loop
 - 4: PID D component of inner loop
 - 8: PID F component of inner loop
 - 16: PID P component of outer loop
 - 32: PID I component of outer loop
 - 64: PID D component of outer loop
 - 128: PID F component of outer loop
 - 256: Roll Axis
 - 512: Pitch Axis
 - 1024: Yaw Axis

**Note 1**: To apply scaling you need to combine at least one PID component and Axis. You can also control multiple parameters by single channels. For example to control inner P and D on Pitch and Roll axes you need to use value 771 as dimentions (1 + 2 + 256 + 521),

**Note 2**: the scale is not uniform, in neutral position it is always 100 percent to represent original config value, if minimum is negative, then neutral position represents 0.

**Note 3**: inner loop is main loop for acro mode, outer loop refers to level mode.

### Mixer

There are few predefined mixers that can be used out-of-the box. You can choose it by setting paramter `mixer_type`

```
set mixer_type QUADX
```

Currently allowed are only:
 - QUADX (default)
 - QUAD1234 (untested)
 - TRI (untested)

You can debug mixer configuration with `mixer` command

```
# mixer 
set mix_outputs 4
set mix_0 1 0 -100
set mix_1 1 1 -100
set mix_2 1 2 100
set mix_3 1 3 100
set mix_4 2 0 100
set mix_5 2 1 -100
set mix_6 2 2 100
set mix_7 2 3 -100
set mix_8 3 0 -100
set mix_9 3 1 100
set mix_10 3 2 100
set mix_11 3 3 -100
set mix_12 4 0 100
set mix_13 4 1 100
set mix_14 4 2 100
set mix_15 4 3 100
set mix_16 0 0 0
```

### Custom Mixer

To activate custom mixer you need to set mixer type to custom and declare number of outputs.

```
set mixer_type CUSTOM
set mix_outputs 4
```

Then apply your custom rules
```
set mix_0 0 0 0
set mix_{index} {src} {dst} {rate}
```
 - `{index}`: rule index
 - `{src}`: mixer source channel
 - `{dst}`: output channel (0-index)
 - `{rate}`: mix rate in percent

**Note**: you need to add a termination rule at the end, to inform mixer to stop processing. In this case set `src` argument to 0. When mixer encounters such a rule, it stops processing and ignore next rules. You can add maximum 64 rules.

Mixer Sources:
 - 0: null for termination rule
 - 1: stabilized roll
 - 2: stabilized pitch
 - 3: stabilized yaw
 - 4: thrust
 - 5: rc roll
 - 6: rc pitch
 - 7: rc yaw
 - 8: rc thrust
 - 9: rc aux 1
 - 10: rc aux 2
 - 11: rc aux 3
