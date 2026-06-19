---
name: coding-patterns
description: C++ standards, sensor driver patterns, naming conventions, and anti-patterns for esp-uav-fc. Enforce before writing or modifying any driver or subsystem.
type: reference
layer: persistent
---

# Coding Patterns

## Language Standard

This project targets **C++17** (confirmed by PlatformIO `build_flags`). The codebase does not yet use C++20/23 features. Do not introduce C++20 concepts, ranges, or coroutines without explicit user approval — ESP-IDF toolchain support is incomplete.

## Model Dependency Injection

**Every subsystem receives `Model&` in its constructor. No globals for state.**

```cpp
// CORRECT
class FooSensor : public BaseSensor {
public:
    FooSensor(Model& model) : _model(model) {}
private:
    Model& _model;
};

// WRONG — do not use
extern Model gModel;
```

`Model` owns all config (`ModelConfig`) and runtime state (`ModelState`). Access config via `_model.config.*` and state via `_model.state.*`.

## Sensor Driver Pattern

New sensor drivers go in `lib/Espfc/src/Device/`. Follow the existing interface:

```cpp
// GyroDevice interface — implement all pure virtuals
class MyGyroXYZ : public GyroDevice {
public:
    int begin(BusDevice* bus) override;
    int begin(BusDevice* bus, uint8_t addr) override;
    DeviceType getType() const override { return GYRO_MY_XYZ; }
    int readGyro(VectorInt16& v) override;
    int readAccel(VectorInt16& v) override;
    void setDLPFMode(uint8_t mode) override;
    int getRate() const override;
    void setRate(int rate) override;
    bool testConnection() override;
};
```

Register new devices in `SensorManager` bus-scan, not in ad-hoc `begin()` calls elsewhere.

**Checklist for every new sensor driver:**
- [ ] Extends the appropriate `*Device` base (`GyroDevice`, `BaroDevice`, `MagDevice`)
- [ ] Uses `BusDevice*` (polymorphic I2C/SPI) — never hard-codes `Wire` or `SPI`
- [ ] `begin()` may block (init only); `read()` must be non-blocking
- [ ] No `delay()` or `delayMicroseconds()` in `read()` — see HW-06
- [ ] Guarded with `#ifndef UNIT_TEST` around hardware includes
- [ ] Added to `GyroDeviceType` / `BaroDeviceType` / etc. enum and `getNames()`

## Naming Conventions

| Thing | Convention | Example |
|-------|-----------|---------|
| Classes | PascalCase | `GyroSensor`, `BusSPI` |
| Methods | camelCase | `readGyro()`, `setRate()` |
| Enums | UPPER_SNAKE_CASE | `GYRO_MPU6050`, `FILTER_NOTCH` |
| Config structs | PascalCase + `Config` suffix | `GyroConfig`, `FilterConfig` |
| Private members | `_camelCase` | `_model`, `_gyro`, `_rpm_q` |
| Macros / target defs | `ESPFC_*` | `ESPFC_SPI_CS_GYRO` |
| Manager classes | PascalCase + `Manager` | `SensorManager`, `SerialManager` |

## Filter Usage

Use `Utils::Filter` with `FilterConfig` — never hand-roll biquad coefficients inline.

```cpp
// Configure a notch at 200 Hz, Q=300
FilterConfig cfg(FILTER_NOTCH, 200, 300);
cfg = cfg.sanitize(loopRate); // clamp freq to Nyquist
Utils::Filter f;
f.init(cfg, loopRate);
// in hot path:
float out = f.update(in);
```

For dynamic notch, use `DynamicFilterConfig` and `FFTAnalyzer`/`FreqAnalyzer` — do not bypass the existing infrastructure.

## Stats Instrumentation

Wrap any new hot-path code with `Stats::Measure` so CPU load shows up in `stats` CLI:

```cpp
Utils::Stats::Measure measure(_model.state.stats, COUNTER_GYRO_READ);
```

Add a new `COUNTER_*` enum entry in `Stats.h` if needed.

## Anti-Patterns to Avoid

| Anti-pattern | Why forbidden | Alternative |
|-------------|--------------|-------------|
| `virtual` in ISR/fast path | Flash cache miss, hard fault | Direct call, template dispatch |
| `delay()` in `read()` | Blocks gyro loop | FreeRTOS notify or state machine |
| Hard-coded GPIO numbers | Breaks multi-target builds | Use `ESPFC_*` macros from Target headers |
| `new`/`delete` in hot path | Heap fragmentation, non-deterministic | Static allocation, member arrays |
| `std::string` in hot path | Heap alloc | `const char*`, `FPSTR()`, `F()` macros |
| Accessing `Model` from two cores without sync | Race condition | `AtomicQueue`, FreeRTOS mutex |
| Adding state to `Espfc.h` directly | Breaks DI pattern | Add to `ModelState` or subsystem class |
| Skipping `--no-verify` on pre-commit | Bypasses CI checks | Fix the underlying lint/test failure |

## FAST_CODE_ATTR Usage

```cpp
// Mark any function on the gyro/PID hot path
int FAST_CODE_ATTR GyroSensor::read() { ... }
int FAST_CODE_ATTR GyroSensor::filter() { ... }
```

On `UNIT_TEST` builds, `FAST_CODE_ATTR` expands to nothing — safe to use unconditionally.

## Test Pattern (native)

Tests live in `test/`. Use Unity macros. Compile-time guard pattern:

```cpp
#ifndef UNIT_TEST
#include "esp_timer.h" // hardware-only header
#endif
```

Run tests with `pio test -e native` — no hardware required.
