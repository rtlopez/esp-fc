# GPS Setup

ESP-FC supports u-blox M8, M9, F9, and M10 GPS modules via UART.

## Hardware Connection

1. Connect GPS TX to ESP32 RX pin
2. Connect GPS RX to ESP32 TX pin
3. Connect VCC (3.3V or 5V depending on module) and GND

## Configure Serial Port

In the `Ports` tab, Disable MSP function and enable GPS on the UART connected to your GPS module. Set baud rate to `115200` or `230400` for 25Hz update rate.

## Enable GPS Feature

In the `GPS` tab, enable **GPS** or use CLI:

```
set feature_gps 1
save
reboot
```

## Basic GPS Configuration

Go to CLI tab and configure basic GPS settings:

```
# Minimum satellites required for valid fix
set gps_min_sats 8

# Set home only once (0 = update home on each arm)
set gps_set_home_once 1

save
reboot
```

> [!NOTE]
> - To see all CLI parameters, type `get gps` in the CLI.
> - After changing GPS configuration, you need to reboot the board for changes to take effect.

## GNSS Configuration (Advanced)

Some modules support multiple GNSS constellations and dual-band (L1+L5) for better accuracy. Configure via CLI:

## Quick Preset Modes

```
# Mode 0: Auto (use individual constellation flags) - DEFAULT
set gps_gnss_mode 0

# Mode 1: GPS only (maximum compatibility)
set gps_gnss_mode 1

# Mode 2: GPS + GLONASS (good for high latitudes)
set gps_gnss_mode 2

# Mode 3: GPS + Galileo (best accuracy in Europe)
set gps_gnss_mode 3

# Mode 4: GPS + BeiDou (optimized for Asia-Pacific)
set gps_gnss_mode 4

# Mode 5: All constellations (maximum satellites, best accuracy, not all modules support it)
set gps_gnss_mode 5

save
reboot
```

> [!NOTE]
> Some modules are not capable to track all constellations at the same time. In this case module initialization may fail.

## Dual-Band Configuration

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

## Individual Constellation Control

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

## Verification

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

## Troubleshooting

### No GPS Fix
1. Check antenna has clear sky view (away from carbon fiber, metal)
2. Enable more constellations: `set gps_gnss_mode 5`
3. Wait 2-3 minutes for initial fix (TTFF - Time To First Fix)
4. Verify baud rate matches GPS module (115200 or 230400)

### GPS Not Detected
1. Check wiring (TX/RX crossed between GPS and FC)
2. Verify serial port configuration in `Ports` tab
3. Try different baud rates (9600, 38400, 57600, 115200, 230400)
4. Check boot logs for `GPS ERROR` or `GPS DET` messages

### Low Satellite Count
1. Move to open area with clear sky view
2. Enable all constellations: `set gps_gnss_mode 5`
3. Check antenna connection
4. Wait longer for satellites to be acquired

#### Slow Fix Time

1. Enable more constellations: `set gps_gnss_mode 5`
2. Ensure SBAS is enabled: `set gps_enable_sbas 1`
3. Check for clear sky view (no buildings, trees blocking)
4. First fix always takes longer (cold start), subsequent fixes are faster
