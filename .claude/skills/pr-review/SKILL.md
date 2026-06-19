---
name: pr-review
description: Review pull requests against ticket intent, spec/plan traceability, standards, and validation evidence using GitHub CLI.
---

# PR Review Workflow

This workflow supports a **two-pass review model**:

1. **Developer preflight review**: produce/update `review.md` locally; do not post to GitHub by default.
2. **Lead gate review**: validate final quality and optionally post review comments to GitHub.

The workflow is **ticket-aware** and prefers resolving context from PR diff paths (including
`spec.md` and `plan.md`) before using branch/title heuristics.

## Runtime Configuration (AGENTS optional)

`AGENTS.md` defaults are optional inputs, not required dependencies.

Resolve pre-context defaults before Step 1, then finalize context-specific values after Step 1:

- **`expected_base_branch`** (pre-context expectation), in this order:
  1. Explicit user-provided base branch (if runner supports it).
  2. `AGENTS.md` Default Git Branch (if present).
  3. Environment: `DEFAULT_GIT_BRANCH` or `GIT_DEFAULT_BRANCH`.
  4. Remote symbolic default:
     ```bash
     git symbolic-ref --short refs/remotes/origin/HEAD 2>/dev/null | sed 's#^origin/##'
     ```
  5. First existing branch in: `main`, `master`, `trunk`, `develop`.
  6. Final fallback: `main` (with warning).

- **`effective_base_branch`** (context-specific branch actually used for diff/review):
  - In `local-branch-diff` context: `effective_base_branch = expected_base_branch`
  - In `github-pr` context: `effective_base_branch = PR baseRefName`

If `AGENTS.md` is missing or lacks these values, continue with the fallbacks above and emit a
non-blocking warning in output.

---

## Prerequisites

### GitHub CLI

```bash
gh --version
gh auth status
```

If authentication fails, run:

```bash
gh auth login
```

`gh` commands may require execution outside the sandbox in constrained environments.

---

## Step 0: Determine Review Mode

Modes:

- **`preflight`** (developer-self-review): default output is chat + `review.md` only.
- **`gate`** (lead final review): can post to GitHub after confirmation.

**Mode selection order:**

1. Explicit user input (`/pr-review <PR> --mode preflight|gate`) if supported by the runner.
2. If PR context is available, compare viewer login and PR author login:
   ```bash
   viewer_login="$(gh api user --jq '.login')"
   pr_author_login="$(gh pr view [PR_NUMBER] --json author --jq '.author.login')"
   ```
   - If `viewer_login == pr_author_login`, default to `preflight`.
   - Otherwise default to `gate`.
3. If PR context is not available yet, default to `preflight` (supports pre-PR local review).

Display detected mode before review starts.

---

## Step 1: Identify Review Context

Determine whether review should run from an existing GitHub PR or local branch diff.

### 1.1 If PR number provided

- Use `github-pr` context with provided PR number.

### 1.2 If no PR number provided

1. Attempt current-branch PR:
   ```bash
   gh pr view --json number,title,state,baseRefName,headRefName
   ```
2. If found, use `github-pr` context.
3. If not found:
   - If mode is `preflight`, use `local-branch-diff` context:
     - Base ref: `origin/[effective_base_branch]`
     - Head ref: `HEAD`
     - Head SHA:
       ```bash
       git rev-parse HEAD
       ```
   - If mode is `gate`, stop with remediation:
     - "No PR found for this branch. Create/open the PR first, or run preflight mode."

### 1.3 Finalize base branch after context selection

- If context is `github-pr`, set `effective_base_branch` to PR `baseRefName`.
- If context is `local-branch-diff`, set `effective_base_branch` to `expected_base_branch`.

---

## Step 2: Fetch Review Context Data

### 2.1 `github-pr` context

```bash
# Metadata
gh pr view [PR_NUMBER] --json number,title,body,state,author,baseRefName,headRefName,headRefOid,additions,deletions,changedFiles,files,commits,reviewDecision,labels,milestone,createdAt,updatedAt

# Diff
gh pr diff [PR_NUMBER]

# Checks
gh pr checks [PR_NUMBER]
```

Capture:

- PR metadata and changed file list.
- Current head SHA (`headRefOid`) for freshness tracking.
- CI/check status.

Warn if PR `baseRefName` differs from `expected_base_branch`.

### 2.2 `local-branch-diff` context (preflight without PR)

```bash
# Ensure base is available
git fetch origin [effective_base_branch]

# Changed files
git diff --name-only origin/[effective_base_branch]...HEAD

# Diff
git diff origin/[effective_base_branch]...HEAD

# Head SHA
git rev-parse HEAD
```

Capture:

- Changed file list and diff from `origin/[effective_base_branch]...HEAD`.
- Current head SHA for freshness tracking.
- CI/check status as `N/A (no PR checks yet)`.

If diff is empty, stop early with:
- "No changes to review against origin/[effective_base_branch]."

---

## Step 3: Resolve Ticket Context (Diff-First)

### 3.1 Discover ticket artifact candidates from changed file list

From changed files, detect directories matching:

- `artifacts/tickets/*/ticket.md`
- `artifacts/tickets/*/spec.md`
- `artifacts/tickets/*/plan.md`
- `artifacts/tickets/*/review.md`

### 3.2 Resolve primary ticket directory

1. If exactly one ticket directory found: use it.
2. If multiple directories found: ask user which directory is in scope for this review.
3. If none found: fallback to ticket extraction from branch/title/body (`[PROJECT]-[N]`) and map to
   `artifacts/tickets/{ID}` (or slug variant if unique).

### 3.3 Load artifacts

From resolved directory, load in this order:

1. `ticket.md` (if present)
2. `spec.md` (preferred)
3. `plan.md` (preferred)
4. existing `review.md` (if present)

If `spec.md` or `plan.md` is missing:

- **Warn explicitly** (traceability gap)
- Continue with reduced-confidence review

---

## Step 4: Build Traceability Review Matrix

Review should prioritize **intent alignment** over generic lint-style feedback.

### 4.1 Extract IDs

From `spec.md` (if present): extract and map IDs such as:

- Success: `S###`
- Functional/non-functional: `F###`, `N###`
- Edge cases/dependencies: `E###`, `D###`
- Design/contracts/models/validation: `C###`, `A###`, `M###`, `V###`
- Testing intent: `U###`, `I###`, `T###`

From `plan.md` (if present): extract and map:

- Execution steps: `P###`
- File touchpoints: `W###`
- Checkpoints: `K###`
- Commit points: `X###`

### 4.2 Correlate with review-context evidence

For each relevant requirement/step, evaluate:

- Is there matching implementation evidence in changed files?
- Is there matching validation evidence (tests/checks/commands)?
- Is there scope creep (changes not mapped to ticket/spec/plan)?

### 4.3 Status rubric

Use:

- `Met`
- `Partial`
- `Missing`
- `Unverifiable`

### 4.4 Review dimensions checklist

Apply these dimensions while reviewing each changed file:

- **Ticket completeness**: acceptance criteria and DoD intent are satisfied (from `ticket.md/spec.md/plan.md`,
).
- **Architecture and scope**: changes are in the correct layer/files and avoid unrelated scope creep. Use the rule→area map in `.claude/context/coding-standards.md` to select the relevant `.claude/rules/` file for the changed area (e.g. gyro/PID → `01-architecture-flight-stack.md`, device drivers → `02-hardware-constraints.md`). Cite the specific rule when a change diverges from it; project rules take precedence over the general catalog.
- **Correctness and safety**:
  - **Control flow**: trace conditionals, guards, and loops — verify that guard
    predicates match the assumptions of the code they protect (e.g. an outer
    check tests "any" but the inner loop assumes "all").
  - **Boundary / off-by-one**: check boundary values, empty inputs, and
    single-element cases for loops, ranges, and string operations.
  - **State consistency**: verify that shared or accumulated state (variables,
    files, environment) is initialized before use and not clobbered between
    iterations or branches.
  - **Failure propagation**: confirm that error/exit codes, exceptions, and
    falsy returns propagate correctly through the call chain — no silently
    swallowed failures.
  - **Security surface**: identify injection vectors (shell, SQL, template),
    unvalidated external input, and credential/secret exposure.
- **Testing and validation**:
  - Tests exist for new/changed behavior and cover both happy path and error paths.
  - Tests verify actual logic, not mock behavior — mocks are acceptable only for
    external dependencies that cannot be called during review.
  - Edge cases identified in the correctness analysis (see §4.4 "Correctness and safety") have corresponding
    test cases or explicit justification for exclusion.
- **Error handling and operability**: failure paths are handled with actionable behavior.
- **Documentation and clarity**: user-facing docs/comments are accurate where behavior changed.

### 4.5 File-by-file review loop

For each changed file:

1. **Inspect the diff hunk(s)** for what changed. The diff is the primary signal for
   scope and intent of the change.
2. **Read the full file** (using your repository file-browsing tools, not just the diff view). The diff tells you
   *what* changed; the full file tells you *what surrounds and interacts with* the change.
   - Identify what the changed lines **depend on** (callers, imports, shared state above
     the hunk) and what **depends on them** (downstream consumers, return-value users
     below the hunk).
   - For non-code files containing embedded code blocks (e.g. bash/python in markdown),
     apply the same correctness dimensions from 4.4 to each code block as if it were a
     standalone script.
3. **Apply relevant checklist dimensions** from 4.4.
4. **Adversarial reasoning**: for each non-trivial change, construct at least one concrete
   failure scenario — pick a realistic input (empty, partial, adversarial) and mentally
   trace it through the changed code path to its output. If the scenario produces incorrect
   behavior, record it as a finding.
5. **Intent verification**: for each diff hunk, identify what the change is trying to
   achieve (from ticket/spec/plan context, commit messages, or code structure) and verify
   the implementation fully achieves it under all realistic conditions — not just the
   happy path. Flag gaps where the mechanism is incomplete or where preconditions are not
   guaranteed by callers.
6. **Verify findings against codebase reality**: before recording a finding, confirm it
   is genuine — check whether the issue is handled elsewhere in the file or codebase,
   whether the current implementation is intentional (comments, commit messages, spec),
   and whether the finding is technically correct for this project's stack and conventions.
   Drop findings that don't survive verification.
7. Record findings with severity and actionable remediation.
8. Mark finding `in_diff` as `false` when issue lines are not part of the PR diff
   (summary-only posting).

After completing the per-file loop:

9. **Cross-file consistency**: when one changed file defines or changes a data shape, API,
   config key, or convention, verify that every other changed file that consumes it is
   consistent. Flag mismatches as findings.

---

## Step 5: Staleness Check for Existing `review.md`

If `review.md` exists, parse metadata:

- reviewed PR number
- reviewed head SHA
- reviewed timestamp
- mode and reviewer

Compare saved `reviewed_head_sha` vs current review head SHA (`PR headRefOid` for `github-pr`,
`git rev-parse HEAD` for `local-branch-diff`):

- If equal: mark review as **current**.
- If different: mark review as **stale** and produce a **follow-up delta** for new commits.

---

## Step 6: Produce Findings and Verdict

Severity levels:

- 🔴 Critical: security, correctness, data-loss risk, missing core acceptance criteria.
- 🟡 Major: architecture/traceability/test gaps likely to cause defects.
- 🟢 Minor: optional improvements and non-blocking cleanups.

Verdict options:

- `APPROVED`
- `CHANGES_REQUESTED`
- `BLOCKED`
- `COMMENT`

### 6.1 Ticket completeness assessment

When ticket context is available, explicitly report:

- Acceptance criteria coverage (`Met`/`Partial`/`Missing`/`Unverifiable`) with evidence.
- DoD/quality gate coverage from available artifacts.
- Scope assessment:
  - in-scope implemented
  - out-of-scope additions
  - required items still missing

If ticket context is unavailable, continue with reduced-confidence warning and code-focused review.

### 6.2 Finding object contract

Normalize findings before writing `review.md` and before any GitHub posting:

```json
{
  "id": 1,
  "path": "relative/path.ext",
  "line": 123,
  "severity": "critical|major|minor",
  "body": "Actionable finding text with evidence and suggested fix",
  "in_diff": true,
  "side": "RIGHT|LEFT",
  "start_line": 120
}
```

Field rules:

- Required: `id`, `path`, `severity`, `body`, `in_diff`.
- `line` required for inline comments; omit for summary-only findings.
- `side` defaults to `RIGHT` unless referencing deleted code.
- `start_line` optional for multi-line inline comments.
- Findings with `in_diff=false` must still appear in `review.md` and summary output.
- Each finding body must include **concrete evidence**: the specific code path, input
  scenario, test name, or CI output that proves the issue. Bare assertions
  ("logic is incorrect", "missing error handling") without a traced example are not
  acceptable findings.

---

## Step 7: Write or Update `review.md` (Tracked Artifact)

Write to:

- `artifacts/tickets/{ID}/review.md` (same directory as `spec.md` and `plan.md`)

Maintain a **single rolling file** with latest state.

### `review.md` structure

```md
# PR Review

## Metadata
- Ticket Directory: artifacts/tickets/...
- PR: #123 | N/A (preflight without PR)
- PR URL: ... | N/A
- Mode: preflight|gate
- Reviewer: ...
- Reviewed At: 2026-03-04T...
- Reviewed Head SHA: abc123...
- Previous Reviewed Head SHA: ... (if follow-up)
- Review Freshness: current|stale-follow-up
- Review Context Source: github-pr|local-branch-diff
- Expected Base Branch: {expected_base_branch}
- Diff Base: origin/{effective_base_branch} | N/A
- Sources:
  - ticket.md: ...
  - spec.md: ...
  - plan.md: ...

## Context Resolution
- How ticket/artifacts were detected (diff-first/fallback)
- Warnings (missing spec/plan, multi-ticket ambiguity resolved, etc.)

## Strengths
- [What is well-implemented: sound design decisions, good test coverage, clean
  separation, effective error handling — be specific with file:line references]

## Traceability Summary
- Requirements evaluated: N
- Met: N
- Partial: N
- Missing: N
- Unverifiable: N

## Findings
### Critical
- ...

### Major
- ...

### Minor
- ...

## Scope Assessment
- In-scope changes:
- Potential scope creep:

## CI / Validation Evidence
- Check status summary
- Test evidence summary

## Verdict
- APPROVED | CHANGES_REQUESTED | BLOCKED | COMMENT
- Required actions:

## Follow-up Delta (only when stale)
- New commits since last review:
- Newly introduced findings:
- Resolved findings:
```

---

## Step 8: GitHub Posting Policy

### Preflight mode (default)

- Do **not** post to GitHub by default.
- Present findings in chat and `review.md`.
- Only post if user explicitly asks.

### Gate mode

- Present summary and proposed inline comments.
- Ask confirmation before posting.
- Show numbered findings and allow exclusions before posting:
  - empty input: post all eligible inline comments
  - `none`: post summary only
  - `cancel`: do not post to GitHub
  - `1,3,5` (or ranges): exclude selected items from inline posting
- If approved, post via `gh api` review endpoint (include `commit_id`).

---

## Step 9: Optional Inline Review Posting

If posting is requested (and `github-pr` context exists):

1. Get repo slug:
   ```bash
   gh repo view --json owner,name --jq '"\(.owner.login)/\(.name)"'
   ```
2. Ensure current head SHA:
   ```bash
   gh pr view [PR_NUMBER] --json headRefOid --jq '.headRefOid'
   ```
3. Build payload file (prefer `--input` JSON file).
4. Post review:
   ```bash
   gh api repos/{owner}/{repo}/pulls/{PR_NUMBER}/reviews \
     --method POST \
     --input /tmp/pr-review-payload.json
   ```
5. Handle edge cases:
   - comments on lines not in diff: summary-only
   - multi-line comments: include `start_line` + `line` when both are in diff
   - large review volume (>50 inline comments): truncate to top-priority inline findings, keep all in summary
6. Fallback strategy:
   - first attempt: full inline payload
   - on partial line-position failures: remove only invalid inline items and retry once
   - on posting failure: post summary-only comment and keep `review.md` as canonical artifact

If current context is `local-branch-diff` (no PR):
- Skip inline posting.
- Keep output in chat and `review.md`.

---

## Error Handling

- **No PR found in preflight**: continue with `local-branch-diff` context.
- **No PR found in gate mode**: stop with remediation (`gh pr list --state open` or create PR).
- **No ticket context**: continue with explicit reduced-confidence warning.
- **Missing `spec.md`/`plan.md`**: continue with traceability-gap warning.
- **Expected vs PR base mismatch**: continue review with PR base as `effective_base_branch`, but keep
  mismatch warning explicit.
- **No changed files**: stop early with a no-op review note.
- **CI failing/pending**: continue review, but call out CI status in verdict rationale.
- **Inline position mismatch**: move affected findings to summary-only and retry once.
- **GitHub post failure**: keep `review.md` as canonical output and provide fallback summary comment.

---

## Quick Reference

```bash
/pr-review
/pr-review 42
```

Recommended use:

1. Developer runs `/pr-review` (preflight) before asking for lead review.
2. Lead runs `/pr-review` (gate) before merge.
3. Both passes update `artifacts/tickets/{ID}/review.md` and compare against PR head SHA freshness.
