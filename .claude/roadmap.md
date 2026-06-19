# Project PHI — Firmware Roadmap

> **Project PHI** is a proprietary fork of [esp-fc](https://github.com/rtlopez/esp-fc) (MIT) targeting a custom 65mm brushed quadcopter PCB (ESP32-S3-WROOM-1U, coreless 615 motors via N-MOSFET).
> This file tracks planned features not yet in the codebase. When a feature ships, move it to a git commit note or ADR — don't leave stale entries here.

---

## Target Hardware (Project PHI PCB)

The PCB uses **non-default pin assignments** configured via CLI/EEPROM after first flash.
`TargetESP32s3.h` defaults are NOT the active pin map for this board.

| Signal | Target default (TargetESP32s3.h) | Project PHI PCB (CLI override) |
|--------|----------------------------------|-------------------------------|
| SPI CS (gyro) | 8 | 4 |
| SPI SCK | 12 | 1 |
| SPI MISO | 13 | 3 |
| SPI MOSI | 11 | 2 |
| Motor 0 | 39 | 10 |
| Motor 1 | 40 | 11 |
| Motor 2 | 41 | 12 |
| Motor 3 | 42 | 13 |

Motor output protocol: **Brushed PWM** (low-side N-MOSFET IRLML6344), NOT DSHOT/ESC.
RC protocol: **ESP-NOW** (50 Hz, built-in, failsafe), NOT CRSF/ELRS.

---

## Phase 0 — Done ✅
- Flash + pin mapping via CLI
- ACRO mode flying
- ESP-NOW RC receiver (`InputEspNow.cpp` present)
- SPL06-001 barometer (`BaroSPL06.h` present)

---

## Phase 1 — In Progress / Near-term

### ICM-42688-P Gyro Driver
**Status:** Missing — no driver in `lib/Espfc/src/Device/`  
**What's needed:** New `GyroICM42688.h` implementing `GyroDevice` interface (SPI, like ICM-20602 but different register map).  
**Rules to follow before implementing:** 02-hardware-constraints (IRAM, SPI rules), 03-coding-patterns (sensor driver checklist), 04-business-logic-decisions (add enum value to `GyroDeviceType` — append only).  
**Reference:** [ICM-42688-P datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)

### PARAM System + Python Bridge
**Status:** Planned  
**What's needed:** MSP-based parameter read/write from Python (ground station / tuning scripts). Enables automated PID tuning and blackbox analysis without Betaflight Configurator.

### Blackbox Integration
**Status:** Connect/ has MSP/blackbox stubs — validate logging works end-to-end on this PCB.

---

## Phase 2 — AI Integration

### Jetson UART Bridge
**Status:** Planned  
**What's needed:** UART interface (likely SERIAL_1 or SERIAL_2) accepting setpoint overrides from a companion Jetson module. Needs failsafe watchdog — if UART goes silent > N ms, revert to RC input.  
**Risk:** Must not block gyro/PID task. Use FreeRTOS queue, not polling in hot path.

### Failsafe Watchdog
**Status:** Planned  
**What's needed:** Autonomous kill / hover command if both ESP-NOW RC AND Jetson UART lost simultaneously.

---

## Phase 3 — Optical Flow & Advanced Modes

### VL53L1X ToF Driver
**Status:** Missing — no driver in `lib/Espfc/src/Device/`, no RangeFinder interface exists  
**What's needed:** I2C driver (`Device/RangeVL53L1X.h` or similar), new `RangeDevice` base class, integration with altitude hold controller.  
**Rules:** Follows same pattern as `BaroDevice` for a new sensor category. Needs ADR before adding new device class hierarchy.

### Optical Flow (MicoAir MTF-01)
**Status:** Planned  
**What's needed:** UART-based optical flow module on `J_UART` port (JST-SH 4-pin). Enables position hold indoors without GPS.

### Custom Flight Modes
**Status:** Planned  
**Candidates:** Position hold (optical flow + ToF), auto-land (ToF descent), AI-guided waypoint.
