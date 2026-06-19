---
argument-hint: "[$001|feature-slug] [feature description]"
---

# Spec: [Feature Name]

**File:** `artifacts/tickets/[TICKET-ID or $001]/spec.md`
**Task ID:** $001 (or feature slug)
**Created:** YYYY-MM-DD
**Status:** Draft | Review | Approved | Implemented

---

## Overview

- Brief summary of the feature/task
- Problem statement and goals

## Success Criteria

Success criteria (use format: **S001-SHORT-NAME**):
- **S001-SHORT-NAME**: [Criterion]
- **S002-SHORT-NAME**: [Criterion]

## Requirements Analysis

**Functional Requirements** (use format: **F001-SHORT-NAME**)
- List each functional requirement with a unique ID
- Expected behavior and outcomes

**Non-Functional Requirements** (use format: **N001-SHORT-NAME**)
- Performance, security, maintainability, usability

**Dependencies** (use format: **D001-SHORT-NAME**)
- External dependencies (services/libraries)
- Internal dependencies (modules/data/other tickets)
- Constraints and limitations

**Edge Cases** (use format: **E001-SHORT-NAME**)
- Boundary conditions, error scenarios, exceptional situations

**Out of Scope**
- Explicit exclusions for this ticket/task (no new ID category)
- When helpful, reference other work by ID (e.g., PROJ-456, $002)

## Technical Approach

- High-level strategy
- Technology/library choices with justification
- Integration points with existing code

### Related ADRs

- List ADRs that govern this spec (e.g., `ADR-012 - Event Schema Versioning`)
- If a new ADR is required, add it here after creation

### Code Quality Considerations (Coding Standards IDs)

- Use `.claude/context/coding-standards.md` as the single entrypoint.
- List the key IDs that apply to this spec (general + language-specific as
  applicable), for example:
  - General: Y### / Z###
  - Project rules: relevant `.claude/rules/` rule ID (e.g. HW-01, HW-06)
- Note where they apply (components/APIs/models) and why.

## Detailed Component Design

For each major component/module (use format: **C001-SHORT-NAME**):

- **C001-COMPONENT-NAME**: Component name and purpose
  - Responsibilities
  - Public interface (use format: **A001-API-NAME** for each key API)
  - Internal data structures (if applicable)
  - Key algorithms/flows (use format: **L001-ALGORITHM-NAME** for complex algorithms)
  - Error handling approach
  - **Follows:** Y### IDs
  - **Avoids:** Z### IDs

## Data Model

For each data structure (use format: **M001-SHORT-NAME**):

- **M001-MODEL-NAME**: Purpose
  - Fields/properties with types
  - Validation rules (use format: **V001-VALIDATION-RULE**)
  - Relationships to other models
  - State/immutability approach
  - **Follows:** Y### IDs
  - **Avoids:** Z### IDs

## Testing Strategy

**Unit Test Scenarios** (use format: **U001-SHORT-NAME**)
- Each unit test scenario with ID
- What is tested + expected outcome
- Link to requirements (e.g., validates F001)

**Integration Test Scenarios** (use format: **I001-SHORT-NAME**)
- Components tested together + expected integration behavior
- Link to requirements

**Test Data Requirements** (use format: **T001-SHORT-NAME**)
- Sample/fixture data needed

## Risk Assessment

For each risk (use format: **R001-SHORT-NAME**):
- Impact, probability, mitigation
- Related requirements/components

## Open Questions

For each open question (use format: **Q001-SHORT-NAME**):
- **Q001-QUESTION-TOPIC**: Question/ambiguity
  - Context: why it matters
  - Possible answers: 2-4 concrete options with trade-offs
  - Who decides: user/stakeholder
  - Impact: effect on implementation/testing/architecture
