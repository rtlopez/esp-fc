# CLAUDE.md — esp-uav-fc

## Project Overview
ESP32-based UAV flight controller firmware (Betaflight-compatible) built with PlatformIO + Arduino framework in C++17. Targets ESP32, ESP32-S3, ESP32-S2, ESP32-C3, RP2040. Core library lives in `lib/Espfc/`.

## Architecture Map
```
src/main.cpp          — Arduino entry (setup/loop), multicore task setup
lib/Espfc/src/
  Espfc.h/cpp         — Top-level orchestrator; owns all subsystems
  Model.h             — Central state passed to every component (config + runtime)
  ModelConfig.h       — Persistent config structs
  ModelState.h        — Runtime-only state structs
  Control/            — PID, rates, fusion, actuator
  Sensor/             — Accel, gyro, baro, mag, voltage wrappers
  Device/             — Low-level hardware drivers
  Output/             — Mixer, motor output (PWM/DSHOT)
  Connect/            — MSP protocol, CLI, blackbox, telemetry
  Rc/                 — CRSF/ELRS receiver
  Utils/              — Filters, math, timers, ring buffers
lib/EscDriver/        — ESC protocol library (DSHOT, Oneshot, etc.)
lib/AHRS/             — Kalman/Madgwick/Mahony attitude estimators
test/                 — Unity unit tests, compile to native x86
```

## Common Commands
```bash
# Build
pio run -e esp32s3          # recommended target
pio run -e esp32            # ESP32 classic
pio run -e native           # native (for tests only)

# Flash
pio run -e esp32s3 -t upload

# Test (runs on native, no hardware needed)
pio test -e native

# All CI targets
pio run                     # builds all envs

# Docker (if no local PlatformIO)
docker-compose run pio pio run -e esp32s3
```

## Code Conventions
- **Model dependency injection**: every subsystem receives `Model&` in its constructor; never use globals for state.
- `IRAM_ATTR` / `FAST_CODE_ATTR` on any function called from the gyro or PID task — timing-critical code must stay in IRAM.
- Manager pattern for multi-device coordination (`SerialManager`, `SensorManager`); direct driver classes live in `Device/`.
- Enums: `UPPER_SNAKE_CASE`. Classes: `PascalCase`. Methods: `camelCase`. Config structs end with `Config`, e.g. `GyroConfig`.
- New sensor drivers go in `lib/Espfc/src/Device/`, registered via bus-scan in `SensorManager`.

## Gotchas & Warnings
- **Partition table**: always use `partitions_4M_nota.csv` for 4MB flash builds. Changing it invalidates OTA slots and SPIFFS storage.
- **OTA merge**: ESP32 variants run `merge_firmware.py` post-build to produce `firmware_0x00.bin` (bootloader + partitions + app). Non-ESP32 targets skip this — don't treat the app binary alone as flashable from address 0x0.
- **Multicore**: on ESP32, gyro ISR/read runs on APP core (priority 24); PID runs on PRO core. Never block in the gyro task or use `delay()` — use FreeRTOS task notifications. RP2040 dual-core is experimental; test changes on single-core first.
- **Betaflight MSP**: only a subset of Betaflight v10.10 MSP commands are implemented. Configurator will silently ignore or reset unsupported fields — verify via CLI `dump`, not just the GUI.
- **Performance budget**: gyro loop target is 1.6–3.2 kHz. Check `stats` via CLI after changes that touch the hot path. CPU load > 70% causes loop overruns.
- **No PSRAM**: S3 build explicitly disables PSRAM (`-DBOARD_HAS_PSRAM` is absent). Don't allocate large buffers assuming PSRAM is available.
- **`UNIT_TEST` guard**: hardware-specific code must be guarded with `#ifndef UNIT_TEST` so tests compile on native without mocking every peripheral.

## Git & Workflow
- Branches: `feature/`, `fix/`, `docs/` prefixes against `master`.
- PRs must pass CI (all 7 platform builds + native tests) before merge.
- Version tags trigger artifact uploads in CI — don't tag unless a release is intended.

## Pointers
- Hardware wiring & pin assignments: [docs/wiring.md](docs/wiring.md)
- CLI parameter reference (100+ params): [docs/cli.md](docs/cli.md)
- Setup, calibration, first-flight checklist: [docs/setup.md](docs/setup.md)
- ESP-NOW wireless receiver setup: [docs/wireless.md](docs/wireless.md)
- Build/flash/docker workflow details: [docs/development.md](docs/development.md)
- All PlatformIO build environments & flags: [platformio.ini](platformio.ini)
- Planned features & Project PHI hardware context: [.claude/roadmap.md](.claude/roadmap.md)

## Context Enforcement

**Before editing any code in this repository**, load and apply the rules from `.claude/rules/` in this order:

| File | When to load |
|------|-------------|
| [.claude/rules/01-architecture-flight-stack.md](.claude/rules/01-architecture-flight-stack.md) | Any change touching gyro ISR, filters, PID loop, mixer, or EscDriver |
| [.claude/rules/02-hardware-constraints.md](.claude/rules/02-hardware-constraints.md) | Any change to Device drivers, GPIO pins, SPI/I2C bus code, or IRAM placement |
| [.claude/rules/03-coding-patterns.md](.claude/rules/03-coding-patterns.md) | Any new class, sensor driver, or interface |
| [.claude/rules/04-business-logic-decisions.md](.claude/rules/04-business-logic-decisions.md) | Any change to a public interface, persisted enum, MSP command, or config layout — **append an ADR first** |
| [.claude/rules/05-session-loop.md](.claude/rules/05-session-loop.md) | After completing any debugging task involving I2C/SPI/DSHOT/filters — **append a Lesson Learned** |

These rules form the project's Persistent/Semantic Memory layer. They prevent recurring mistakes (IRAM violations, blocking in `read()`, enum reordering) and keep interface decisions consistent across sessions.
