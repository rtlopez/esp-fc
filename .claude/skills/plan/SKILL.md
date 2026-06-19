---
name: plan
description: Converts ticket specs (spec.md) into test-driven implementation plans (plan.md) with ordered steps, traceability, and validation. Use when the user runs /plan or needs an actionable implementation plan.
---

# Plan Skill

## Execution Model

Invoke the **`technical-lead`** subagent (`.claude/agents/technical-lead.md`) to do the actual work. This isolates codebase exploration from the main conversation and saves tokens.

The subagent will:
1. Read spec.md and grep the codebase independently
2. Identify files to create/modify and decide implementation sequence
3. Write `artifacts/tickets/{ID}/plan.md`
4. Present a summary and gate on human approval before /implement can start

Pass to the subagent: the path to the spec file (`artifacts/tickets/{ID}/spec.md`).

**Prerequisite:** spec.md must exist and be approved by the user.

---

Converts a ticket `spec.md` into a test-driven implementation plan (`plan.md`) using the unified P/W/K/X format.

---

## Workflow

### 1. Determine Input Source

**Preferred input:** `artifacts/tickets/{ID}/spec.md`

If no spec is provided, ask for the spec path or ticket ID, or recommend running `/spec` first.

### 1b. Consult ADR Index

Read `.claude/context/coding-standards.md` to identify which project rule files govern the areas
this plan touches. Cite the specific rule file and rule ID in the relevant **P### steps** — not
only in Related Documentation — so `/implement` reviews them at the point of change.

| Area touched | Load rule file |
|-------------|---------------|
| Gyro/filters/PID/mixer/EscDriver | `.claude/rules/01-architecture-flight-stack.md` |
| Device drivers, GPIO, SPI/I2C, IRAM | `.claude/rules/02-hardware-constraints.md` |
| New classes, sensor drivers, interfaces | `.claude/rules/03-coding-patterns.md` |
| Public interfaces, enums, MSP, config | `.claude/rules/04-business-logic-decisions.md` |

- *In-flight ADRs:* a draft ADR relevant to this ticket may exist under `docs/adr/`;
  check there and cite it if found.

### 2. Parse Spec File

**Read and extract:**

1. **Traceability anchors:** S###, F###, N###, E###, C###, A###, M###, U###, I###
2. **Open questions:** Q### that must be resolved before committing to plan steps
3. **Interfaces and contracts:** APIs, models, validation rules to reflect in P### steps
   - If a step introduces/changes an API/model/validation, include brief interface/contract notes in the
     P### step (do not defer this to execution; it drives correct implementation and tests).

### 3. Generate Plan Document (Unified Format)

Use the template from:
- [PLAN-TEMPLATE.md](PLAN-TEMPLATE.md) - Canonical P/W/K/X plan structure

**Work Packages (optional)**

If the plan has many steps or spans multiple areas, group related P### steps using headings:

```markdown
### Work Package: [NAME]
```

Work Packages are for readability only. Execution remains P###-driven (checkboxes, dependencies,
K### checkpoints, X### commit points). Do not introduce a second task ID scheme (e.g., `T01`), and do not
reference a separate task template.

---

## Output Location

**Save plan to:**
- Ticket: `artifacts/tickets/$001/plan.md` (optionally `artifacts/tickets/$001-short-slug/plan.md`)
- Ad-hoc: `artifacts/tickets/$001/plan.md` (optionally `artifacts/tickets/$001-short-slug/plan.md`)

The plan must live next to its corresponding `spec.md`.

---

## Validation Checklist

Before finalizing plan, verify:

- [ ] All success criteria (S###) and functional requirements (F###) are covered by P### steps
- [ ] Each P### step references spec IDs and includes concrete validation
- [ ] Each step cites relevant coding standards IDs via `.claude/context/coding-standards.md`
  - General: Y### / Z###
  - Project rules: relevant `.claude/rules/` file and rule ID (e.g., HW-01, HW-06)
- [ ] Steps in a rules-governed area cite the governing rule file
- [ ] Dependencies correctly mapped
- [ ] Test coverage is comprehensive (U### and I### coverage)
