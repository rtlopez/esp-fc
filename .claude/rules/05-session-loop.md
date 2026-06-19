---
name: session-loop
description: Self-Correction Loop. Append Lessons Learned after each session involving I2C/SPI debugging, sensor integration, or real-time issues.
type: reference
layer: persistent
---

# Self-Correction Loop

## Purpose

After each session where a bug was found, a driver was debugged, or an integration failed, append an entry below. Future sessions MUST read this file before touching I2C/SPI drivers, filter code, or the PID loop.

**Format each entry as:**

```
### [YYYY-MM-DD] <Short title>
**Symptom**: What observable failure occurred.
**Root Cause**: The actual bug or misunderstanding.
**Fix**: What change resolved it.
**Rule Updated**: Which rule file (if any) was updated as a result.
**Watch For**: Trigger conditions that indicate the same issue is recurring.
```

---

## Lessons Learned

### [2026-04-08] Initial rules system bootstrap
**Symptom**: N/A — baseline session.
**Root Cause**: No persistent memory across sessions led to repeated re-derivation of project context.
**Fix**: Created `.claude/rules/` with architecture, hardware constraints, coding patterns, ADR log, and this session loop.
**Rule Updated**: All five rule files created.
**Watch For**: If a future session proposes `virtual` functions in ISR context, or `delay()` in sensor `read()`, the rules were not loaded — reload `02-hardware-constraints.md` and `03-coding-patterns.md`.

---

## Recurring Failure Patterns (Reference)

These patterns have appeared in esp-fc forks across multiple projects. Check these first when debugging:

### I2C / SPI Hangs at Boot
- **Cause A**: Wrong I2C address. `MPU6050` defaults to `0x68`; AD0 high → `0x69`. Always call `testConnection()` and log the detected address.
- **Cause B**: SPI CS line not driven high before `begin()`. CS must be HIGH before the first transaction.
- **Cause C**: Soft-I2C on S3 (`ESPFC_I2C_0_SOFT`) is slower than hardware I2C. If clock stretching occurs, the bit-bang loop may time out.

### Gyro Loop Overrun (CPU > 70%)
- **Cause A**: New sensor `read()` added a blocking operation (I2C poll, `delay()`).
- **Cause B**: Dynamic Notch FFT window too large for loop rate. `DYN_NOTCH_COUNT_MAX = 6`; verify `_dyn_notch_denom` is > 1.
- **Cause C**: New filter stage not using `FAST_CODE_ATTR` — function body is in flash, causing cache stalls.
- **Check**: `stats` CLI command; look at `COUNTER_GYRO_READ` and `COUNTER_GYRO_FILTER`.

### DShot Telemetry RPM Not Updating
- **Cause**: `_rpm_enabled` requires both `config.gyro.rpmFilter.harmonics > 0` AND `config.output.dshotTelemetry == true`. Checking only one condition is the common mistake.

### Native Test Build Failing
- **Cause**: Hardware header included outside `#ifndef UNIT_TEST` guard. The native target has no ESP-IDF headers.
- **Fix**: Wrap every `#include "esp_*.h"` or Arduino peripheral include.

### Config Layout Corruption After Firmware Update
- **Cause**: Enum value inserted mid-list in a persisted enum (e.g., `GyroDeviceType`). Old EEPROM data maps to wrong enum value.
- **Fix**: Always append to enum tail, never insert. Bump config version if a break is unavoidable and add migration code in `Storage`.
