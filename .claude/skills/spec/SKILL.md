---
name: spec
description: Create ticket-level specifications (spec.md) with fully labeled requirements, component designs, and test strategy. Use when the user runs /spec or needs a detailed, implementation-ready spec for a ticket.
---

# Spec Skill

## Execution Model

Invoke the **`business-analyst`** subagent (`.claude/agents/business-analyst.md`) to do the actual work. This isolates spec context from the main conversation and saves tokens.

The subagent will:
1. Read project context independently (CLAUDE.md, rules, roadmap)
2. Analyze the feature request
3. Write `artifacts/tickets/{ID}/spec.md`
4. Present a summary and gate on human approval before /plan can start

Pass to the subagent: the user's feature description and the target ticket ID (or `$NNN`).

---

Create a ticket-level specification for implementation work, saved as `spec.md` under the ticket directory.

## Output Location

- Ticket: `artifacts/tickets/$001/spec.md` (optionally `artifacts/tickets/$001-short-slug/spec.md`)
- Ad-hoc: `artifacts/tickets/$001/spec.md` (optionally `artifacts/tickets/$001-short-slug/spec.md`)

## Workflow

1. **Gather context**
   - If `artifacts/tickets/{ID}/ticket.md` exists (from `/pull-ticket`), extract ticket ID, title,
     description, acceptance criteria, and dependencies.
   - If a legacy `ticket.md` exists in repo root, it can be used as a fallback.
   - Otherwise: ask for minimum context (goal, key flows, inputs/outputs, constraints, acceptance criteria).

2. **Review existing repo context**
   - Identify adjacent code/modules and existing patterns.
   - Identify integration points and affected files at a high level.
   - **Consult the ADR index:** Treat `docs/adr/README.md` (the canonical ADR catalogue) as the
     primary index. Identify ADRs whose titles or decisions relate to the ticket being specified and
     pre-populate the "Related ADRs" section of the spec with candidates.
     - *Fallback / augmentation:* If `docs/adr/README.md` is missing, empty, unparseable, or
       yields zero candidates, scan `docs/adr/ADR-<NNN>-*.md` directly. Also check
       `docs/adr/` for any in-flight draft ADR relevant to this ticket. If ADR files are
       found that are not referenced in the README, note in the spec that the ADR index may be out of date.

3. **Resolve identifier and output path**
   - Use `$NNN` or a short feature slug for ad-hoc tasks (e.g. `$001` or `icm42688-driver`).
   - If no task identifier is provided, suggest the next `$NNN` by scanning `artifacts/tickets/`.

4. **Write the spec**
   - Use the template at `.claude/skills/spec/SPEC-TEMPLATE.md`.
   - Preserve the labeling scheme and ID format exactly as defined in the template.
   - Reference coding standards and anti-patterns by Y/Z IDs (see guidance below).
   - Only reference ADRs in the spec; record architectural decisions in ADRs, not in spec content.

5. **Address open questions**
   - If the spec contains Q### questions that block planning, get decisions before finalizing the plan phase.

## Template

Use `.claude/skills/spec/SPEC-TEMPLATE.md` as the single source of truth for structure, labels, and output format.

## Authoring Guidance

### Architectural Decisions and ADRs

- Specs should *reference* relevant ADRs in the "Related ADRs" section.
- If a new architectural decision is required, create an ADR under `docs/adr/`.
  - Keep the decision details in the ADR; only link or list the ADR in the spec.
  - Use `adr` skill to draft the ADR before finalizing the spec.

### Code Quality Standards (Y/Z)

Coding standards and anti-patterns are canonical under `.claude/context/coding-standards.md`

When describing components/APIs/models, explicitly cite the most relevant IDs:
- **Guidelines:** Y###-NAME (e.g., Y100-SINGLE-RESPONSIBILITY, Y103-DEPENDENCY-INJECTION)
- **Anti-patterns:** Z###-NAME (e.g., Z102-GOD-OBJECTS, Z200-LEAKY-ABSTRACTION)
- **Project rules:** Reference the relevant `.claude/rules/` file (e.g., HW-01, HW-06 from `02-hardware-constraints.md`)

## Workflow Integration

After completing this spec:

1. Save to `artifacts/tickets/{ID}/spec.md`
2. Resolve any blocking **Q###** questions (record decisions in the spec)
3. Create the plan using: `/plan artifacts/tickets/{ID}/spec.md`
4. Plan is saved to: `artifacts/tickets/{ID}/plan.md`

## Spec Authoring Rules (Do Not Skip)

- **ID format:** LETTER + 3 DIGITS + DASH + SHORT-NAME (e.g., F001-PARSE-CSV)
- **Numbering:** increment sequentially within each category; start each category at 001
- Be specific; avoid vague requirements
- Prefer traceability: reference F/C/M/E/U/I IDs across sections
- Use Y/Z IDs to make standards expectations explicit

## Recording Q### Decisions

If you have open questions, keep the spec self-contained by recording decisions in this format:

```markdown
### Q001-QUESTION-TOPIC
**Question:** [Original question]

**Decision:** [Chosen answer]

**Rationale:** [Why this decision was made]

**Impact:** [How this affects implementation/testing]
```

## ID Reference Guide

*Spec IDs (defined in this spec):*
- **S001**: Success Criteria
- **F001**: Functional Requirements
- **N001**: Non-functional Requirements
- **D001**: Dependencies
- **E001**: Edge Cases
- **C001**: Components
- **A001**: APIs
- **L001**: Algorithms
- **M001**: Data Models
- **V001**: Validation Rules
- **U001**: Unit Tests
- **I001**: Integration Tests
- **T001**: Test Data
- **R001**: Risks
- **Q001**: Questions

*Code Quality IDs (cite in this spec):*
- **Y100-Y9999**: Guidelines
- **Z100-Z9999**: Anti-patterns to avoid
