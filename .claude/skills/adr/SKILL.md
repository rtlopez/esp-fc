---
name: adr
description: Create Architecture Decision Records (ADRs) under docs/adr/ using the ADR template.
---

# ADR Skill

Create an Architecture Decision Record for project-wide or cross-cutting decisions.

## Output Location

- `docs/adr/ADR-<NNN>-<short-title>.md`

## Workflow

1. **Gather decision context**
   - Decision title, problem statement, constraints, and alternatives.
   - Related specs/plans or tickets that triggered the decision.

2. **Determine ADR number and filename**
   - Scan `docs/adr/` for existing `ADR-<NNN>-*.md` files.
   - Use the next sequential 3-digit number (e.g., 001, 002).
   - Create a short, lowercase, hyphenated slug (3-7 words).

3. **Write the ADR**
   - Use the template at `.claude/skills/adr/ADR-TEMPLATE.md`.
   - Keep it concise: context, decision, rationale, consequences.
   - Avoid implementation details (those belong in specs/plans).

4. **Cross-reference**
   - Link related ADRs and specs/plans in the ADR.
   - Reference the ADR from any relevant spec using the "Related ADRs" section.

5. **Refresh the ADR index**
   - `docs/adr/README.md` is the canonical ADR index (table of ADRs plus thematic groupings).
   - After adding a new ADR file, update the table so titles, status, and dates stay coherent.

## Conventions

- ADRs live under `docs/adr/`
- File naming follows `ADR-<NNN>-<short-title>.md`
- Prefer one decision per ADR
- `docs/adr/README.md` is the canonical entry point used by other skills (`spec`, `plan`) for ADR discovery
- Project rules in `.claude/rules/` are operational constraints; ADRs explain the *why* behind decisions

## Related Standards

If the decision establishes or constrains coding practices, cite relevant Y/Z IDs in
"Related Guidelines" and use the `coding-standards` skill for reference.
