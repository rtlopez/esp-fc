# ADR-001 — Establish `.claude/rules/` Layered Context System

| Field | Value |
|-------|-------|
| **Date** | 2026-04-08 |
| **Status** | Accepted |
| **Supersedes** | — |

## Context

Adding new sensors and interfaces to the esp-uav-fc fork requires consistent architectural guardrails across multiple AI-assisted development sessions. Without persistent context, the same mistakes recur between sessions: IRAM violations, blocking calls in `read()`, enum reordering in persisted types, and contradictory interface proposals.

## Decision

Create `.claude/rules/` with five rule files covering: flight stack architecture, hardware constraints, coding patterns, business-logic decision log (this ADR system), and a session self-correction loop. Add a Context Enforcement table to `CLAUDE.md` specifying which rule file to load before which type of edit.

Additionally, create `.claude/skills/` with workflow skills (spec, plan, implement, adr, pr-description, pr-review) and `docs/adr/` as the canonical location for firmware ADRs.

## Consequences

**Positive**
- Consistent coding patterns enforced across AI sessions
- ADR history prevents contradictory interface proposals across sessions
- Hardware constraints (IRAM, GPIO, SPI/I2C) prevent common errors

**Negative**
- Slight overhead: relevant rule files must be read at session start before editing

**Neutral**
- Rule files are Markdown — can be updated by the developer as the project evolves
- ADRs are Markdown — human-readable, diff-friendly, no tooling required

## Affected Files

- `.claude/rules/` (all five rule files)
- `.claude/skills/` (workflow skills)
- `docs/adr/` (this ADR and future ones)
- `CLAUDE.md`
