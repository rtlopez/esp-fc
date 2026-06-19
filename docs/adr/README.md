# Architecture Decision Records

ADRs for the esp-uav-fc / Project PHI firmware fork. Each file records one architectural decision: context, decision, and consequences.

Use `/adr` to create a new ADR. New ADRs go in `docs/adr/ADR-NNN-short-title.md`.

## Index

| ID | Title | Status | Date |
|----|-------|--------|------|
| [ADR-001](ADR-001-rules-system.md) | Establish `.claude/rules/` layered context system | Accepted | 2026-04-08 |

## When is an ADR required?

See `.claude/rules/04-business-logic-decisions.md` for the full trigger list and pre-change checklist.

Short version: any time you change a **public interface, persisted enum, MSP command, or config layout** — write an ADR first.
