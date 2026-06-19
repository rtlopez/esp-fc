# Implementation Plan: [Feature Name]

**File:** `artifacts/tickets/[TICKET-ID or $001]/plan.md`
**Spec:** `artifacts/tickets/[TICKET-ID or $001]/spec.md`
**Task ID:** $001 (or $001 for ad-hoc)
**Created:** YYYY-MM-DD
**Status:** Not Started | In Progress | Completed

---

## Progress Summary

**Status:** Work has not started (run `/implement` to begin execution)
**Last Updated:** YYYY-MM-DD
**Completed Steps:** 0 / [total]

---

## 0. Preconditions, Open Questions, and Risks

List any items that block accurate planning or safe execution.

**Preconditions**
- e.g., access to service X, feature flag Y enabled, migration prerequisite

**Open questions (from spec Q###)**
- **Q001-...**: [What must be decided before implementation]

**Risks (from spec R###)**
- **R001-...**: [Risk + mitigation pointer]

## 1. Implementation Strategy

- Overall sequencing and critical path
- Architectural constraints — cite the governing ADR(s) from `docs/adr/`
- Parallel work opportunities (if any)

## 1b. Related Documentation (Optional)

- Spec: `artifacts/tickets/[TICKET-ID or $001]/spec.md`
- ADRs: `docs/adr/` (link the relevant ADR filenames; `docs/adr/` for any in-flight draft)
- Blueprints / PRD references (if applicable)
- Similar existing code (paths)

## 2. Requirements Traceability

List the spec requirements that this plan covers:

- **F001-NAME**: How it will be implemented (P### references)
- **N001-NAME**: How it will be addressed
- **E001-NAME**: Where it is tested/handled
- Deferred items (explicitly called out)

## 2b. Testing Strategy (High-Level)

- TDD expectations for this ticket (if applicable)
- Which test types matter (unit/integration/manual) and why
- Where checkpoints (K###) are placed to de-risk the work

## 3. Implementation Steps (P###)

Each step must be checkable and traceable to spec IDs.

Optionally group related steps using **Work Packages** (headings only; no new ID system).

### Work Package: [NAME]

- [ ] **P001-STEP-NAME**: Step description
  - ...

- [ ] **P001-STEP-NAME**: Step description
  - **Purpose**: What this step accomplishes
  - **What to do** (optional): 2-6 concrete sub-steps
  - **Interface/contract notes** (optional): brief schema/contract notes when changing A###/M###/V###
  - **Implements**: F###, C###, M###, E### (as applicable)
  - **Files affected**:
    - **W001**: `path/to/file.ext` (create/modify/delete)
    - **W002**: `path/to/test_file.ext` (create/modify)
  - **Code quality**:
    - **Follows**: Y### (general), project rule IDs (e.g. HW-01)
      - See `.claude/context/coding-standards.md` (entrypoint to full catalogs)
    - **Avoids**: Z### (general), anti-patterns from `.claude/rules/`
    - **ADRs/Rules**: governing rule file + ID for this area (see `.claude/context/coding-standards.md`)
  - **Dependencies**: P### prerequisites (if any)
  - **Validation**: U### / I### test IDs + concrete commands to run

- [ ] **P002-NEXT-STEP**: ...

## 4. File Change Summary (W###)

- [ ] **W001**: `path/to/file.ext`
  - **Action**: Create/Modify/Delete
  - **Purpose**: What this file does
  - **Related specs**: C###, M###, A###, etc.
  - **Modified in steps**: P### list

## 5. Integration Points (G###) (Optional)

- **G001-INTEGRATION-NAME**: Integration description
  - **Connects**: components/modules
  - **Contract**: A### from spec
  - **Implemented in**: P### list

## 6. Testing Checkpoints (K###)

- [ ] **K001-CHECKPOINT-NAME**: Checkpoint description
  - **After step**: P###
  - **Run**: concrete commands (tests/lint/typecheck)
  - **Validates**: U### / I### (and/or F###)

## 7. Commit Points (X###)

Commits should align with meaningful plan milestones.

- [ ] **X001-COMMIT-NAME**: Commit description
  - **After step**: P###
  - **Includes files**: W### list
  - **Message template**: `feat: ... ([TICKET-ID])`

## 8. Final Validation & Acceptance

- Functional validation: F### list
- Non-functional validation: N### list
- Edge cases: E### list
- Manual testing steps (if any)
- Acceptance criteria: S### list

## 9. Rollback Plan

- Safe stop points: X### list
- Revert procedure: what to undo if needed

---

## Implementation Log (Populate During /implement)

Use these sections to keep the plan self-contained as execution progresses.

## Execution Gaps vs Spec

- Gaps discovered between spec and implementation reality (link to spec IDs)

## Lessons Learned

- Short notes that will help future tickets avoid mistakes

## Key Decisions and Deviations From Plan

- Changes made during implementation (what changed, why, impact)

## Validation Results

- Commands executed + pass/fail notes (tie back to K### checkpoints)

## Plan Authoring Rules (Do Not Skip)

- **ID format:** LETTER + 3 DIGITS + DASH + SHORT-NAME (e.g., P001-CREATE-PARSER)
- **Numbering:** increment sequentially within each plan category (P/W/K/X/G), starting at 001
- Every P### step must reference spec IDs (F/C/M/E/U/I), cite Y/Z IDs for code quality, and cite the governing rule file where applicable (see `.claude/context/coding-standards.md`)
- Work Packages are headings only; do not introduce a second task ID scheme (e.g., `T01`)
