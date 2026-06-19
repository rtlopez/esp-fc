---
name: business-logic-decisions
description: Defines when an ADR is required and what to check before proposing any interface, enum, MSP, or config change. ADRs live in docs/adr/.
type: reference
layer: persistent
---

# Business Logic Decisions — Rule

## When is an ADR Required?

Before proposing any change that affects a public interface, enum value, protocol behavior, or persistent config layout, **write an ADR first** in `docs/adr/` using `/adr`.

An ADR is required when:

- Adding or renaming fields in `ModelConfig` / `ModelState`
- Adding a new enum value that is persisted (EEPROM/flash storage)
- Changing the MSP command set or protocol wire format
- Changing how sensors are discovered/registered in `SensorManager`
- Changing the EscDriver interface or DShot timing parameters
- Changing the partition table or OTA behavior

ADRs are append-only. Mark superseded ADRs as `Status: Superseded` and reference the new ADR ID.

**ADR index:** [`docs/adr/README.md`](../../docs/adr/README.md)

---

## Checklist Before Proposing an Interface Change

- [ ] Does this change existing `ModelConfig` fields? → check EEPROM layout, may require version bump
- [ ] Does this add a new `GyroDeviceType` / `BaroDeviceType` enum value? → enum is persisted — **append only, never reorder**
- [ ] Does this change an MSP command ID or payload? → Betaflight Configurator compatibility may break
- [ ] Does this add a new SPI/I2C device? → check CS pin availability, bus speed compatibility
- [ ] Does this change `FAST_CODE_ATTR` function signatures? → re-validate linker placement
