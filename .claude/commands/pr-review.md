# /pr-review - Pull Request Review

Review a pull request against intent, standards, and verification evidence.

## Usage

```bash
/pr-review [PR_NUMBER]
```

## Forwarding

Invoke the `pr-review` skill and follow `.claude/skills/pr-review/SKILL.md`.

## Inputs

- Optional PR number

## Output

- Updated `artifacts/tickets/{ticket-id}/review.md` with review metadata and findings
- Optional inline review comments (typically in lead gate mode)
