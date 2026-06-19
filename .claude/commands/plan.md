# /plan - Create plan.md from spec.md

Convert a ticket spec into an executable implementation plan.

## Usage

```bash
/plan [$001|feature-slug|TICKET-DIR|SPEC-PATH]
```

## Forwarding

Invoke the `plan` skill and follow `.claude/skills/plan/SKILL.md`.

## Inputs

- Ticket identifier, ticket directory, or `spec.md` path

## Output

- `artifacts/tickets/{ID}/plan.md`
