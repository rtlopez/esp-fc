---
name: hardware-constraints
description: Hard rules for ESP32/S3 hardware: IRAM placement, SPI/I2C bus usage, forbidden GPIO pins. Enforce before any driver or ISR change.
type: reference
layer: persistent
---

# Hardware Constraints

## IRAM / Flash Placement Rules

**RULE HW-01 — No virtual calls in ISR/fast-path**
Any function reachable from the gyro task or ISR MUST be annotated `FAST_CODE_ATTR` (maps to `IRAM_ATTR` on ESP32 targets). Virtual function dispatch can trigger a cache miss if the vtable entry or function body is in flash — this causes an exception or hard fault at flight speed.

- Allowed: direct method calls, `static` functions, template resolution at compile time
- Forbidden: `virtual` dispatch, `std::function`, lambda captures with vtable, `new`/`delete` in hot path
- Pattern used in codebase: `int FAST_CODE_ATTR GyroSensor::read()`

**RULE HW-02 — `#ifndef UNIT_TEST` guard for all hardware code**
Every `#include` of ESP-IDF / Arduino peripherals must be wrapped so the native test build compiles without mocking every peripheral. Failure breaks `pio test -e native` and CI.

## SPI Bus Rules

**RULE HW-03 — Never block-wait on SPI in task context**
`BusSPI::transfer()` is synchronous and holds the bus. On ESP32-S3, `SPI1` is used (`ESPFC_SPI_0_DEV SPI1`). Do not call SPI reads from two tasks without an explicit mutex. The gyro read is always on the gyro task — do not add secondary SPI sensors that also read from the PID task.

**RULE HW-04 — Speed constants from `BusSPI`**
Use `BusSPI::SPI_SPEED_NORMAL` (1 MHz) for init/config and `BusSPI::SPI_SPEED_FAST` (16 MHz) for data reads. Never hard-code raw Hz values; adding a sensor? pass the constant, not a literal.

## I2C Bus Rules

**RULE HW-05 — I2C is soft-I2C on ESP32-S3**
`ESPFC_I2C_0_SOFT` is defined on S3. Hardware I2C (`Wire`) is not used. Max rate is 2000 Hz (`ESPFC_GYRO_I2C_RATE_MAX`). Do not exceed this — the soft-I2C driver has no DMA and every byte is bit-banged.

**RULE HW-06 — No `delay()` or `delayMicroseconds()` in sensor drivers**
Blocking calls in `begin()` are tolerated (init only). They are FORBIDDEN in `read()` or any function called from the loop. Use FreeRTOS task delays (`vTaskDelay`) only in non-real-time tasks.

## GPIO Pin Map — DO NOT CHANGE

### ESP32-S3 (`TargetESP32s3.h`)

| Pin | Function | Note |
|-----|----------|-------|
| 0 | — | Strapping: BOOT mode select — leave floating/high |
| 3 | — | Strapping |
| 45 | — | Strapping: VDD_SPI voltage |
| 46 | — | Strapping: ROM messages |
| 19, 20 | USB D−/D+ | JTAG/CDC — do not reassign |
| 26–37 | Flash/PSRAM | Reserved by silicon — NEVER assign |
| 8 | SPI CS Gyro | `ESPFC_SPI_CS_GYRO` |
| 7 | SPI CS Baro | `ESPFC_SPI_CS_BARO` |
| 11 | SPI MOSI | `ESPFC_SPI_0_MOSI` |
| 12 | SPI SCK | `ESPFC_SPI_0_SCK` |
| 13 | SPI MISO | `ESPFC_SPI_0_MISO` |
| 9 | I2C SDA | `ESPFC_I2C_0_SDA` |
| 10 | I2C SCL | `ESPFC_I2C_0_SCL` |
| 39–42 | Motor outputs 0–3 | `ESPFC_OUTPUT_0..3` |
| 6 | PPM input | `ESPFC_INPUT_PIN` |
| 15, 16 | UART1 RX/TX | MSP port |
| 17, 18 | UART2 RX/TX | RC serial |
| 43, 44 | UART0 TX/RX | MSP port |
| 5 | Buzzer | `ESPFC_BUZZER_PIN` |
| 1 | ADC0 (VBAT) | `ESPFC_ADC_0_PIN` |
| 4 | ADC1 (current) | `ESPFC_ADC_1_PIN` |

**ADC scale**: `3.3V / 4096` — 12-bit, 3.3V reference.

## Memory Rules

**RULE HW-07 — No PSRAM assumption**
`-DBOARD_HAS_PSRAM` is absent on S3 build. `ps_malloc` / `heap_caps_malloc(MALLOC_CAP_SPIRAM)` must not be used. All buffers must fit in DRAM. If a new feature needs > ~50 KB of RAM, raise the issue before implementation.

**RULE HW-08 — Partition table is fixed**
Always use `partitions_4M_nota.csv`. Changing the partition CSV invalidates existing OTA slots and SPIFFS storage on every deployed board. Any partition change requires an explicit user decision and full re-flash from address 0x0 using `firmware_0x00.bin`.

## Target Preprocessor Guards

| Macro | Meaning |
|-------|---------|
| `ESPFC_FREE_RTOS` | FreeRTOS available |
| `ESPFC_MULTI_CORE` | Dual-core (ESP32, S3) |
| `ESPFC_DSP` | DSP/FFT available (S3 only) |
| `ESPFC_DSHOT_TELEMETRY` | Bidirectional DSHOT RPM feedback |
| `UNIT_TEST` | Native test build — no hardware |
