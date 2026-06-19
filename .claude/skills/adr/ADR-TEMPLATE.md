# ADR-001: [Short Title of Decision]

**Status**: Proposed | Accepted | Rejected | Deprecated | Superseded
**Date**: YYYY-MM-DD
**Deciders**: [List of people involved in the decision]
**Related ADRs**: [Links to related ADRs if any]

## Context

What is the issue we're facing? What factors are influencing this decision?

Provide enough context so that readers can understand:

- The problem we're trying to solve
- Why we need to make this decision now
- What constraints or requirements we're working within
- What the current situation is

## Decision

What decision have we made?

State the decision clearly and concisely. Use active voice: "We will..." or "We have
decided to..."

Example: "We will use FastAPI as our web framework for building the API layer."

## Rationale

Why did we make this decision?

Explain the reasoning behind the decision:

- What options did we consider?
- What were the pros and cons of each option?
- What factors tipped the balance?
- What assumptions are we making?

### Options Considered

#### Option 1: [Name]

**Pros**:

- Pro 1
- Pro 2

**Cons**:

- Con 1
- Con 2

#### Option 2: [Name]

**Pros**:

- Pro 1
- Pro 2

**Cons**:

- Con 1
- Con 2

### Decision Factors

What made us choose this option?

- Factor 1: Explanation
- Factor 2: Explanation
- Factor 3: Explanation

## Consequences

What are the results of this decision?

### Positive Consequences

- Benefit 1: Description
- Benefit 2: Description
- Benefit 3: Description

### Negative Consequences

- Trade-off 1: Description and how we'll mitigate it
- Trade-off 2: Description and how we'll mitigate it

### Neutral Consequences

- Impact 1: Description
- Impact 2: Description

## Implementation

How will this decision be implemented?

- Step 1: Description
- Step 2: Description
- Step 3: Description

**Timeline**: Expected implementation timeframe

**Dependencies**: What needs to happen first, or what other systems are affected?

## Validation

How will we know if this decision was correct?

- Metric 1: How we'll measure success
- Metric 2: How we'll measure success
- Review Date: When we'll review this decision

## Related Guidelines (Optional)

If this ADR establishes or constrains coding practices, cite relevant standards IDs
from `.claude/context/coding-standards.md`:

**Guidelines (Y###) - what to follow:**
- **Y###-GUIDELINE-NAME**: [Relation to decision]

**Anti-patterns (Z###) - what to avoid:**
- **Z###-ANTI-PATTERN**: [Relation to decision]

## References

- [Link to related documentation]
- [Link to research or external resources]
- [Link to discussions or RFCs]

## Notes

Any additional notes, caveats, or context that doesn't fit elsewhere.

---

## ADR Writing Guidelines

### When to Write an ADR

Write an ADR when:

- Making a significant architectural decision
- Choosing between multiple viable options
- Making a decision that will be hard to reverse
- Making a decision that affects multiple parts of the system
- Setting a precedent for future similar decisions

### ADR Best Practices

1. **Keep it concise**: ADRs should be readable in 5-10 minutes
2. **Be specific**: Include enough detail to understand the decision
3. **Document alternatives**: Show what options were considered
4. **Explain trade-offs**: Be honest about negative consequences
5. **Date your decision**: Include when the decision was made
6. **Update status**: Mark as superseded if a new ADR replaces this one
7. **Link related ADRs**: Create a web of decisions
8. **Review periodically**: Set review dates for important decisions

### ADR Statuses

- **Proposed**: Decision is being discussed, not yet accepted
- **Accepted**: Decision has been made and approved
- **Rejected**: Proposal was considered but not accepted
- **Deprecated**: Decision is no longer relevant
- **Superseded**: Replaced by a new ADR (link to it)

---

*This is a template. Copy it to create new ADRs, naming them ADR-NNN-short-title.md*
