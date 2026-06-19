# Coding Standards - Entry Point

**Goal:** Make it easy for agents to discover and cite coding standards by ID.

This repository uses a general language-agnostic standards pack plus project-specific rules:

- **General (language-agnostic):** `./coding-standards/general/` (Y*/Z*)
- **Project-specific C++17 rules:** `.claude/rules/03-coding-patterns.md`

Use these IDs when citing standards in specs and plans:
- **General guidelines:** Y###-NAME (e.g. Y100-SINGLE-RESPONSIBILITY)
- **General anti-patterns:** Z###-NAME (e.g. Z102-GOD-OBJECTS)

---

## Architecture Decision Records (authoritative)

The standards pack above is the general catalog. The repository's binding **architectural**
decisions live in `.claude/rules/04-business-logic-decisions.md` (ADR log) and the project
rules files. Where a project rule and the general catalog overlap, **the project rule is the
decision of record**.

Before implementing in an area, load the relevant rule file:

| Area | Rule file |
|------|-----------|
| Gyro ISR, filters, PID loop, mixer, EscDriver | `.claude/rules/01-architecture-flight-stack.md` |
| Device drivers, GPIO pins, SPI/I2C, IRAM placement | `.claude/rules/02-hardware-constraints.md` |
| New classes, sensor drivers, interfaces | `.claude/rules/03-coding-patterns.md` |
| Public interfaces, persisted enums, MSP commands, config layout | `.claude/rules/04-business-logic-decisions.md` |
| Post-debugging lessons (I2C/SPI/DSHOT/filters) | `.claude/rules/05-session-loop.md` |

The `/spec`, `/plan`, and `/implement` skills consult these rules; cite the governing rule file
alongside the Y/Z IDs.

---

## Fast Discovery

To see all general standards in one go:

- General full catalog (Y*/Z*): `./coding-standards/general/index-catalog.xml`

Structure guide: `./coding-standards/overview.md`
