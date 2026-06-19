---
name: pr-description
description: Generate a ready-to-use pull request description (pr-description.md) from a branch's commits, diff, and ticket artifacts, conforming to the repository PR template. Use when the user runs /pr-description or needs a PR body to paste into GitHub or hand to another agent (managing-git/gh-cli) opening the PR.
---

# PR Description Skill

Produce a pull request description as a standalone Markdown file derived from the **current branch's
state vs. the base branch**, structurally aligned with the canonical repository PR template at
`.github/pull_request_template.md` (sections, ordering, the `Coverage:` block under Validation,
optional-section comments). This skill **never duplicates** the template — it reads it at runtime
and renders a filled instance.

The output is the canonical artifact: any agent opening the PR (e.g. via `managing-git` or `gh-cli`)
**MUST** consume it with `gh pr create --body-file <pr-description.md>`. It may also be copied and
pasted directly into the GitHub PR UI.

This skill **does not** open or edit a PR — it only writes the description file. If the user wants
the PR opened, defer to `managing-git` / `gh-cli` after this skill completes.

## Output Location

Write to one of the following, in priority order:

1. **Ticket-scoped** (preferred when a ticket directory is resolvable):
   `artifacts/tickets/{ID}/pr-description.md`
2. **Repo-scoped fallback** (no ticket directory resolvable):
   `artifacts/pr-descriptions/{branch-slug}.md`
3. **Explicit override**: if the user provides a path, write there instead.

Always print the absolute output path at the end of the run.

## Runtime Configuration (AGENTS optional)

Resolve these defaults before Step 1. If `AGENTS.md` is missing or lacks values, fall back as
described and emit a non-blocking warning.

- **`expected_base_branch`** — order of resolution:
  1. Explicit user-provided base branch.
  2. `AGENTS.md` "Default Git Branch".
  3. Environment: `DEFAULT_GIT_BRANCH` or `GIT_DEFAULT_BRANCH`.
  4. Remote symbolic default:
     ```bash
     git symbolic-ref --short refs/remotes/origin/HEAD 2>/dev/null | sed 's#^origin/##'
     ```
  5. First existing branch in: `main`, `master`, `develop`, `trunk`.
  6. Final fallback: `main` (with warning).

- **`pr_template_path`** — the single source of truth is the repository PR template:
  `.github/pull_request_template.md`.
  This skill does **not** ship a fallback template — if the file is missing, stop with remediation
  ("`.github/pull_request_template.md` not found — restore the repo PR template before running
  `/pr-description`."). Do not invent or duplicate template content.

---

## Prerequisites

- Working tree on a feature branch (not the base branch itself).
- `git fetch` has been (or can be) run against `origin` to compare against the up-to-date base.

```bash
git --version
git rev-parse --is-inside-work-tree
```

If the current branch equals `expected_base_branch`, stop with remediation:
"Refusing to write a PR description for the base branch. Switch to your feature branch first."

---

## Step 1: Identify Branch Context

```bash
current_branch="$(git branch --show-current)"
git fetch origin "$expected_base_branch"
base_ref="origin/${expected_base_branch}"
head_sha="$(git rev-parse HEAD)"
```

Validate there are commits to describe:

```bash
git rev-list --count "${base_ref}..HEAD"
```

If the count is `0`, stop early:
"No commits between ${base_ref} and HEAD. Nothing to describe."

---

## Step 2: Collect Source Evidence

Use **diff-first** inputs. Commit messages and ticket artifacts are enrichment, not substitutes.

### 2.1 Diff and changed files

```bash
# File list with status (A/M/D/R)
git diff --name-status "${base_ref}...HEAD"

# Stat summary
git diff --shortstat "${base_ref}...HEAD"

# Full diff (capture for review; do not paste raw diff into PR body)
git diff "${base_ref}...HEAD"
```

Group changed paths by **top-level layer** (e.g. `lib/Espfc/src/`, `lib/EscDriver/`, `lib/AHRS/`,
`src/`, `test/`, `docs/`, `.github/`, `platformio.ini`). The grouping drives the "Areas touched" section.

### 2.2 Commit log on the branch

```bash
# Subjects only
git log --pretty=format:'%h %s' "${base_ref}..HEAD"

# Full messages (used to extract scope/intent — never copy verbatim into the body)
git log --pretty=format:'%h%n%s%n%n%b%n---' "${base_ref}..HEAD"
```

Extract Conventional Commit `type:` prefixes to inform the PR title (`feat`, `fix`, `docs`,
`refactor`, `test`, `chore`, `perf`).

### 2.3 Ticket artifacts (diff-first, then branch-name fallback)

Discover candidates from the changed file list:

- `artifacts/tickets/*/ticket.md`
- `artifacts/tickets/*/spec.md`
- `artifacts/tickets/*/plan.md`
- `artifacts/tickets/*/review.md`

Resolution rules:

1. If exactly one ticket directory is touched, use it.
2. If multiple are touched, ask the user which is in scope.
3. If none are touched, extract a ticket token (`KEY-NNN`) from the branch name (preferring
   and map to `artifacts/tickets/{ID}/`. If that directory exists,
   use it; otherwise continue without ticket context.

Load (in order, if present): `ticket.md`, `spec.md`, `plan.md`, `review.md`.

Use these strictly to enrich **Summary**, **Validation**, and risk/rollout content — do not
re-paste their full bodies.

### 2.4 Validation evidence

Search the diff for evidence of executed checks:

- New/changed files under `test/`.
- Build/test commands referenced in commit messages (e.g. `pio test -e native`, `pio run -e esp32s3`).
- CI workflow files under `.github/workflows/` touched by the diff.

Record concrete command(s) the reviewer can re-run.

---

## Step 3: Compose the PR Description

Use the resolved `pr_template_path` as the structural source. If a comment in the template marks a
section as optional (e.g. `<!-- Optional: include only when relevant -->`), only emit that section
when the diff justifies it. Drop the surrounding `<!-- ... -->` comment wrappers in the final body —
the file must be free of unresolved template comments.

### 3.1 Required sections

- **Summary** — 1–3 sentences. State **what changed** and **why**. Lead with intent (from
  `spec.md` / ticket title when available), not a file enumeration.
- **Areas touched** — short bulleted list grouped by layer/package. Reference top-level paths.
  Avoid listing every file; one bullet per package/area is usually enough.
- **Validation** — keep the template's `Coverage:` block. Fill the three sub-fields:
  - `Command:` the exact validation command(s) (e.g. `pio test -e native` or `pio run -e esp32s3`).
  - `Result:` the outcome from the most recent run, or an explicit `(to be run by reviewer)`
    placeholder if not yet executed in this session.
  - `Notes:` follow-up checks, coverage gates, or manual validation steps.

### 3.2 Optional sections (include only when justified by the diff)

- **Reviewer guide** — include when the change spans multiple unrelated layers, has a non-obvious
  reading order, or hinges on a single load-bearing decision worth flagging up-front.
- **Risk / rollout** — include when the change touches infra, migrations, feature flags, public
  contracts, vendored code, or anything with a non-trivial rollback story.
- **Hardware evidence** — include when pin assignments, PCB config, or electrical design changed.
  Reference relevant wiring doc sections.
- **Security / privacy** — include when auth, secrets handling, PII, dependency surface, or
  cross-tenant boundaries change.

### 3.3 Title

Derive a PR title (printed alongside the file for convenience, not written into the body):

- Format: `<type>: <imperative summary> (<TICKET-KEY>)`
  - `<type>` from the dominant Conventional Commit prefix on the branch.
  - `<TICKET-KEY>` from resolved ticket context; omit when no ticket is resolvable.
- ≤72 characters.

### 3.4 Style rules

- Imperative, present-tense ("Add", "Refactor", "Fix") in Summary and bullets.
- No raw diff hunks or shell transcripts in the body.
- No ticket-internal jargon that is not defined or linked.
- No emojis (project convention).
- No AI co-author trailers and no "Generated with …" footers (see Hard Rules).

---

## Step 4: Validate the Output

Before declaring success, the produced `pr-description.md` MUST pass these checks:

1. All required template sections (`Summary`, `Areas touched`, `Validation`) are present and
   contain non-placeholder content.
2. No unresolved template comments (`<!-- ... -->`) remain in the body.
3. No blank/filler bullets (`-`, `- ...`, `- TBD`, `- TODO`, `- N/A`).
4. No AI co-author / "Generated with" lines anywhere in the body.
5. The `Coverage:` block under `Validation` has all three sub-fields filled (`Command:`, `Result:`,
   `Notes:`). `(to be run by reviewer)` is an acceptable explicit `Result:` value when validation
   was not executed locally; an empty value is not.
6. Optional sections, if included, contain content — never just headings.

If any check fails, fix the body and re-validate before writing the final file.

---

## Step 5: Write the Artifact

Write the file at the resolved output path (Step "Output Location"). Then print to the chat:

- Absolute path of the artifact.
- Proposed PR title.
- Resolved `expected_base_branch`.
- Resolved ticket directory (or "none").
- Any warnings (template fallback used, missing `spec.md`/`plan.md`, multi-ticket ambiguity resolved, etc.).

End the run with explicit handoff guidance:

> "Use `managing-git` / `gh-cli` to open the PR with
> `gh pr create --base <base> --title '<title>' --body-file <path>`,
> or paste the file contents into the GitHub PR description."

---

## Hard Rules

- MUST derive content from the **actual diff and commit log**, not assumptions.
- MUST conform to `.github/pull_request_template.md` when present (sections, ordering, the
  `Coverage:` block under Validation).
- MUST keep the file free of unresolved template comments and placeholder bullets.
- MUST NOT add AI co-author trailers (`Co-authored-by: Claude/Cursor/Codex/<any AI>`) or
  "Generated with …" footers — see the project rules in `AGENTS.md` ("No AI co-author
  trailers") and `.claude/rules`.
- MUST NOT open, push, or edit a PR from this skill; that's the job of `managing-git` / `gh-cli`.
- MUST NOT paste full diff hunks, commit bodies, or `spec.md`/`plan.md` content verbatim.

## Anti-patterns

- Listing every changed file in "Areas touched" instead of summarising by package/layer.
- Using the ticket title as the entire Summary instead of describing what the branch actually
  changed.
- Leaving `Coverage: Result:` empty when validation has been run, or filling it with `TBD`.
- Embedding shell transcripts, raw diff output, or large code blocks in the PR body.
- Including optional sections (Risk, UI evidence, Security, etc.) as empty headings to "look
  thorough".

---

## Error Handling

- **On base branch**: stop with remediation message; do not write a file.
- **No commits vs. base**: stop early with the "Nothing to describe" message.
- **PR template missing** (`.github/pull_request_template.md` not found): stop with remediation;
  do not write a file and do not synthesize a replacement template.
- **Multiple ticket directories in diff**: ask the user which is in scope; do not guess.
- **No ticket directory resolvable**: continue without ticket context, omit ticket key from the
  title, and write to `artifacts/pr-descriptions/{branch-slug}.md`.
- **`git fetch` fails (offline)**: continue against the existing local `origin/<base>` ref and warn
  that the base may be stale.

---

## Quick Reference

```bash
/pr-description
/pr-description $001
/pr-description artifacts/tickets/$001
/pr-description --output artifacts/pr-descriptions/my-branch.md
```

Recommended flow:

1. Finish implementation on the feature branch and commit.
2. Run `/pr-description` to produce `pr-description.md`.
3. Hand off to `managing-git` / `gh-cli` to open the PR with `--body-file`, or paste the file into
   the GitHub UI.
