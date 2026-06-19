---
name: architecture-flight-stack
description: Data-flow architecture from gyro ISR through filters, PID loop, and DShot output. Reference before touching any real-time path.
type: reference
layer: persistent
---

# Flight Stack Architecture

## Real-Time Data Flow

```mermaid
architecture-beta
  group isr(server)[Gyro ISR / APP Core pri-24]
  group pid(server)[PID Task / PRO Core]
  group out(server)[Output Stage]

  service gyro_hw(disk)[GyroDevice SPI/I2C] in isr
  service gyro_read(server)[GyroSensor::read FAST_CODE_ATTR] in isr
  service filter_pre(server)[Pre-filter PT1/PT2/PT3] in isr
  service notch_static(server)[Static Notch BiquadDF1] in isr
  service dyn_notch(server)[DynNotch FFTAnalyzer / FreqAnalyzer] in isr
  service rpm_filter(server)[RPM Notch harmonics 1-3] in isr
  service atomic_queue(server)[AtomicQueue / FreeRTOS notify] in isr

  service fusion(server)[Control::Fusion Madgwick/Mahony/Kalman] in pid
  service rates(server)[Control::Rates setpoint calc] in pid
  service controller(server)[Control::Controller inner+outer PID] in pid
  service actuator(server)[Control::Actuator TPA/anti-gravity] in pid
  service mixer(server)[Output::Mixer quad/hex/custom] in pid

  service dshot(disk)[EscDriver DSHOT300/600] in out
  service pwm(disk)[EscDriver PWM/Oneshot] in out
  service telem(disk)[DSHOT Telemetry RPM feedback] in out

  gyro_hw:R --> L:gyro_read
  gyro_read:R --> L:filter_pre
  filter_pre:R --> L:notch_static
  notch_static:R --> L:dyn_notch
  dyn_notch:R --> L:rpm_filter
  rpm_filter:R --> L:atomic_queue
  atomic_queue:R --> L:fusion
  fusion:R --> L:rates
  rates:R --> L:controller
  controller:R --> L:actuator
  actuator:R --> L:mixer
  mixer:R --> L:dshot
  mixer:R --> L:pwm
  dshot:R --> L:telem
  telem:B --> T:rpm_filter
```

## Key Classes & Files

| Layer | Class | File |
|-------|-------|------|
| Orchestrator | `Espfc` | `lib/Espfc/src/Espfc.h` |
| State/Config | `Model` | `lib/Espfc/src/Model.h` |
| Gyro read+filter | `GyroSensor` | `lib/Espfc/src/Sensor/GyroSensor.h` |
| Attitude fusion | `Fusion` | `lib/Espfc/src/Control/Fusion.h` |
| Rate PID | `Controller` | `lib/Espfc/src/Control/Controller.h` |
| Motor mix | `Mixer` | `lib/Espfc/src/Output/Mixer.h` |
| ESC protocol | `EscDriver` | `lib/EscDriver/` |
| Filter primitives | `Filter` | `lib/Espfc/src/Utils/Filter.h` |
| FFT (S3 only) | `FFTAnalyzer` | `lib/Espfc/src/Utils/FFTAnalyzer.hpp` |
| Freq analyzer | `FreqAnalyzer` | `lib/Espfc/src/Utils/FreqAnalyzer.hpp` |

## Filter Chain Detail (GyroSensor::filter)

1. `filter3` — static LPF/PT1 (pre-filter, runs first)
2. `notchFilter[3]` — up to 3 static Notch/BiquadDF1 per axis
3. Dynamic Notch — driven by `FFTAnalyzer` (S3, `ESPFC_DSP`) or `FreqAnalyzer`; `_dyn_notch_denom` decimates at `loopRate/1000`
4. RPM Notch — `harmonics 1-3` at motor eRPM frequencies, fades in via `_rpm_fade_inv`
5. `filter` / `filter2` — post-filter LPF (feeds PID D-term)

## Timing Budget

| Stage | Target | Measured via |
|-------|--------|-------------|
| Gyro loop (SPI) | 250–4000 Hz | `model.state.gyro.timer.rate` |
| Gyro loop (I2C) | 250–2000 Hz | `ESPFC_GYRO_I2C_RATE_MAX 2000` |
| PID loop | same as gyro / `loopSync` | `model.state.loopTimer.rate` |
| CPU budget | < 70% | `stats` CLI command |

## Multicore Layout (ESP32/S3)

```
APP core  (priority 24): gyro ISR read → filter chain → AtomicQueue push
PRO core  (priority  3): queue pop → Fusion → Controller → Mixer → EscDriver
```

On single-core targets (ESP32-C3, RP2040, native) everything runs sequentially in `Espfc::update()`.
