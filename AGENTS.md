# AGENTS.md

Guidelines for AI agents working on **Esp-FC**. Keep changes small, tested and portable.

## Project overview

- Flight controller firmware, C++17, [PlatformIO](https://platformio.org/).
- Firmware targets: `esp32`, `esp32s2`, `esp32s3`, `esp32c3`, `esp8266`, `rp2040`, `rp2350`.
- `native` env is host-only and used for unit tests.
- Arduino dependency must be isolated in Hal namespace
- it is not allowed to include new Arduino libraries, as there is process or removing Arduino dependency

## Repository layout

- `src/main.cpp` – thin entry point, all logic lives in libraries.
- `lib/Espfc/src/` – main firmware library:
  - `Model.h`, `ModelConfig.h`, `ModelState.h` – central state and persisted settings.
  - `Control/`, `Sensor/`, `Input*`, `Output/`, `Rc/`, `Telemetry/`, `Blackbox/`, `Stream/` – feature modules.
  - `Device/` – device drivers, `Hal/` – platform abstraction, `Target/` – board/target definitions.
  - `Utils/` – filters, math, logger, storage, timing helpers.
- `lib/AHRS`, `lib/EscDriver`, `lib/Gps`, `lib/betaflight`, ... – supporting libraries.
- `test/test_<area>/test_<area>.cpp` – unit tests.
- `bin/` – helper scripts, `docs/` – user and dev docs.

## Before you start

- Read the module you are about to change and its neighbours; follow the patterns already there.
- Read `docs/development.md` for the contribution workflow.
- Keep the diff minimal – no drive-by refactors, no unrelated formatting changes.

## Implementing a feature

1. Locate the right module; prefer extending existing abstractions over adding new layers.
2. If the feature needs user settings, add them to `ModelConfig.h` (and defaults/reset paths) and wire them through `Model`.
3. Keep platform-specific code in `Hal/` or `Target/`; feature code stays portable.
4. Wire up CLI / MSP / Blackbox / telemetry only when the feature actually needs it.
5. Add or extend unit tests for the new behaviour.
6. Run the verification commands below before declaring the work done.

## Coding conventions

- `#pragma once` in headers, everything inside `namespace Espfc` (plus sub-namespace matching the directory).
- `.hpp` for declarations, `.cpp` for definitions, `.ipp` for template implementations.
- Private members prefixed with `_` (e.g. `_model`), classes `PascalCase`, methods `camelCase`.
- Formatting is enforced by `.clang-format` (LLVM base, 2 spaces, 120 columns) – never hand-format against it.
- Builds must stay `-Wall` clean.
- Hot paths (gyro loop, ISRs, DShot/serial handling) must not allocate, block or log.
- Respect `IRAM_ATTR` and existing timing/scheduling assumptions in the main loop.
- Write code, names, comments, and documentation in English.
- Keep comments limited. Do not add comments where the code already clearly expresses what it does.

## Portability

- Code must compile for **all** default envs; CI builds each one.
- Guard platform-specific code with the existing macros: `ESP32S2`, `ESP32S3`, `ESP32C3`, `ARCH_RP2040`, `ARCH_RP2350`, `UNIT_TEST`, `NO_GLOBAL_INSTANCES`.

## Testing

- Put tests in `test/test_<area>/test_<area>.cpp`, mirroring the existing style.
- Prefer a failing test first, then the fix/feature, then a passing test.
- Untested changes will not accepted (see `docs/development.md`).

## Verification commands

```
pio test -e native            # unit tests (must pass)
pio run -e esp32s3            # build specific target
pio run                       # build all targets
pio run -t check_format       # code style check - only when asked
pio run -e native -t format   # apply formatting - only when asked
pio check                     # static analysis (cppcheck) - only when asked
```

Without a local PlatformIO install, prefix with `docker compose run --rm espfc`.

## Git & pull requests

- Commit messages: short imperative summary, explain *why* when not obvious.
- A PR must address only one issue or feature.
- A PR must state the problem being solved, provide evidence the feature works, keep tests and static analysis green, and avoid unnecessary changes.
