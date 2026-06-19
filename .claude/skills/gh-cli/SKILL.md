---
name: gh-cli
description: Use the GitHub CLI (`gh`) for auth, PRs, and issues with reliable command patterns (run outside sandbox; prefer --body-file for descriptions to avoid shell escaping problems).
---

# GitHub CLI (`gh`) Skill

Use this skill when the user asks to use `gh` (GitHub CLI): authenticate, create/edit PRs or issues,
query GitHub, or troubleshoot `gh` failures.

## Hard Rules

- MUST run `gh` commands outside sandbox when auth/network is required.
- MUST use `--body-file` for any multi-line PR/issue body.
- MUST re-fetch posted PR body and re-validate after create/edit.
- MUST resolve base branch before `gh pr create`.
- MUST fail fast if explicit template hint cannot be resolved.
- NEVER use `--body` for multi-line content.
- NEVER pass a local file path to `gh issue create --template` for YAML forms.
- NEVER infer base branch from arbitrary feature branch names.

## Non-sandbox requirement

In this environment, `gh` authentication and related operations may fail when run in a sandbox.

When executing `gh` via the agent terminal tool:
- Run `gh` commands **outside the sandbox** (with full system access to network and credentials) so
  authentication can succeed.
- If `gh auth status` / `gh auth login` fails due to sandbox restrictions, rerun the same `gh`
  command with sandboxing disabled.
- Assume **any** `gh ...` command may need this (auth checks, PR creation, API calls).

## Core pattern: draft file + `--body-file`

For any multi-line body, always:
1. Build/update a markdown body file with agent file operations.
2. Submit using `--body-file`.
3. Re-read posted content and validate again.

## Preflight Checklist

Before any `gh pr create`, `gh pr edit`, or `gh issue create`:
1. `gh auth status` succeeds.
2. Base branch is resolved (PR flow only).
3. Template source is resolved (or fallback is chosen explicitly).
4. Body file exists (for markdown body flow).
5. Validation passes.

## Base branch resolution

Resolve `effective_base_branch` in this order:
1. Explicit user-provided base (if supplied).
2. Branch-configured merge base (stacked PR workflows):
   ```bash
   git config --get "branch.$(git branch --show-current).gh-merge-base"
   ```
3. Existing PR base for current branch:
   ```bash
   gh pr view --json baseRefName --jq '.baseRefName'
   ```
4. Repository default branch:
   ```bash
   gh repo view --json defaultBranchRef --jq '.defaultBranchRef.name'
   ```
5. Final fallback (emit warning in output): `develop`, then `main`. This repo's default branch
   is `develop`, so prefer it first when nothing else resolves.

## Template discovery

Use repository file inspection and deterministic order.

### PR template discovery

If `template_hint` is provided:
1. Check explicit path as given.
2. Check common mapped paths:
   - `{hint}`
   - `{hint}.md`
   - `.github/{hint}`
   - `.github/{hint}.md`
   - `.github/PULL_REQUEST_TEMPLATE/{hint}`
   - `.github/PULL_REQUEST_TEMPLATE/{hint}.md`
3. If not found, fail fast and report checked locations.

Without `template_hint`, use first existing path in order:
1. `pull_request_template.md`
2. `docs/pull_request_template.md`
3. `.github/pull_request_template.md`
4. `.github/PULL_REQUEST_TEMPLATE.md`
5. First sorted markdown file under:
   - `PULL_REQUEST_TEMPLATE/`
   - `docs/PULL_REQUEST_TEMPLATE/`
   - `.github/PULL_REQUEST_TEMPLATE/`

### Issue template discovery

If `template_hint` is provided:
1. Check explicit path as given.
2. Check common mapped paths:
   - `{hint}`
   - `{hint}.md`
   - `{hint}.yml`
   - `{hint}.yaml`
   - `.github/ISSUE_TEMPLATE/{hint}`
   - `.github/ISSUE_TEMPLATE/{hint}.md`
   - `.github/ISSUE_TEMPLATE/{hint}.yml`
   - `.github/ISSUE_TEMPLATE/{hint}.yaml`
3. If not found, fail fast and report checked locations.

Without `template_hint`, use first existing path in order:
1. `.github/ISSUE_TEMPLATE/feature_request.md`
2. `.github/ISSUE_TEMPLATE/bug_report.md`
3. First sorted template under `.github/ISSUE_TEMPLATE/` with extension `.md`, `.yml`, or `.yaml`,
   excluding `config.yml`/`config.yaml`
4. `issue_template.md`
5. `docs/issue_template.md`
6. `.github/issue_template.md`

## Fallback bodies when no template is found

### PR fallback body

Use this starter:

```markdown
## Summary
- Describe the change and user impact.

## Changes
- List concrete code/documentation changes.

## Testing
- List executed checks (or explain why not run).
```

### Issue fallback body (feature request default)

Use this starter:

```markdown
## Problem Statement
- What user/developer problem exists today?

## Proposed Solution
- What should change?

## Impact
- Why this matters and who benefits.
```

## Validation

Run these checks on body text before `gh pr create`, `gh pr edit`, or `gh issue create`:
1. Reject unresolved template comments (`<!-- ... -->`).
2. Reject placeholders:
   - blank bullets (`-` / `- `)
   - filler bullets (`- ...`, `- TBD`, `- TODO`, `- N/A`)
3. For each present key section, require meaningful non-placeholder content.
4. If `Type of Change` section is present, require at least one checked checkbox in that section
   scope only (do not count later sections).
5. If source template has YAML frontmatter, strip it before writing the final body file.

## Command blueprints

### PR create/edit (markdown body)

```bash
gh pr create --base "<effective_base_branch>" --title "<title>" --body-file <path>
gh pr edit <number> --title "<title>" --body-file <path>
```

### Issue create (markdown body)

```bash
gh issue create --title "<title>" --body-file <path>
```

### Issue create (YAML form template)

```bash
gh issue create --template "<form-name>"
# or interactive selection:
gh issue create
```

## Decision tree: issue creation

1. Resolve issue template.
2. If template extension is `.yml`/`.yaml`, use `gh issue create --template "<form-name>"`.
3. Otherwise generate/fill markdown body and use `--body-file`.
4. If no template exists, use fallback issue body and `--body-file`.

## PR workflow

1. Resolve base branch using the order above.
2. Resolve PR template (or fallback body).
3. Fill body with concrete details from diff/commits/tests.
4. Validate body.
5. Create or update PR (command blueprint above).
6. Re-fetch posted PR body:
   ```bash
   gh pr view <number> --json body --jq '.body'
   ```
7. Re-validate and repair with `gh pr edit --body-file` if needed.

Retry transient `gh` failures up to 3 attempts before failing.

## Issue workflow

1. Resolve issue template (or fallback body).
2. Follow issue decision tree.
3. Validate markdown body when `--body-file` path is used.

## Anti-patterns

- `gh pr create --body "...multi-line..."`
- `gh issue create --template .github/ISSUE_TEMPLATE/feature_request.yml`
- Skipping post-create PR body verification.
- Using unresolved placeholder text in submitted bodies.

## Completion contract

When finishing a PR/issue task, report:
1. Resolved base branch (PR flow).
2. Resolved template source (or fallback used).
3. Exact `gh` command path used (`--body-file` or `--template`).
4. Created/updated URL or number.
5. Validation status (pre-submit and post-submit for PRs).

## Common operations

### Auth

```bash
gh auth status
gh auth login
gh auth setup-git
```

### API queries

```bash
gh api repos/<owner>/<repo>/pulls --jq '.[].number'
```

## Worked examples

### Example A: PR without template

1. Resolve base branch.
2. Generate fallback PR body.
3. Fill with concrete summary/changes/testing.
4. Validate.
5. `gh pr create --base "<effective_base_branch>" --title "docs: ..." --body-file /tmp/pr-body.md`
6. Re-fetch and re-validate.

### Example B: Issue from YAML form

1. Resolve template `.github/ISSUE_TEMPLATE/feature_request.yml`.
2. Use form name `feature_request.yml` with:
   ```bash
   gh issue create --template "feature_request.yml"
   ```
3. If template name fails, use interactive `gh issue create`.
