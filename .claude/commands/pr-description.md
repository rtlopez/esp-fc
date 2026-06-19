# /pr-description - Generate PR description (pr-description.md)

Generate a pull request description from the current branch's commits, diff, and ticket artifacts,
conforming to `.github/pull_request_template.md`. The output is a Markdown file ready to paste into
GitHub or hand to another agent (e.g. `managing-git` / `gh-cli`) opening the PR.

This command **does not** open or edit a PR.

## Usage

```bash
/pr-description
/pr-description [$001|feature-slug|TICKET-DIR]
/pr-description --output artifacts/pr-descriptions/<branch-slug>.md
```

## Forwarding

Invoke the `pr-description` skill and follow `.claude/skills/pr-description/SKILL.md`.

## Inputs

- Optional ticket identifier or ticket directory (overrides diff-based detection).
- Optional explicit output path.

## Output

- `artifacts/tickets/{ID}/pr-description.md` when a ticket directory is resolvable.
- `artifacts/pr-descriptions/{branch-slug}.md` otherwise.
- Proposed PR title printed alongside the artifact path (not written into the body).
