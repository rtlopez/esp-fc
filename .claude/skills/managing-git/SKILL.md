---
name: managing-git
description: Git operations for ticket-based development including branch creation, commits, pushes, and pull requests. Use when the user needs to create branches, commit changes, push to remote, or create PRs during implementation workflows.
---

# Managing Git Skill

Provides Git operations for ticket-based development, including branch management, commits, and pull
requests.

## Default Variables

Retrieve from `AGENTS.md` (see "Default Variables" section):
- **Default Git Branch**: Base branch for ticket branches (this repo uses `master`)

---

## Operations

### 1. Create Ticket Branch

**Generate branch name from ticket:**

**Format:** `<prefix>/<ticket-key>-<ticket-name-slug>`

**Prefix selection (choose one):**

| Prefix | Use for |
|--------|---------|
| `feat/` | Feature / new capability |
| `refactor/` | Refactor (no functional change) |
| `doc/` | Documentation |
| `test/` | Tests |
| `conf/` | Configuration / tooling |

If unclear, ask the user which prefix they want. Default to `feat/` if they prefer not to decide.

**Sanitization rules:**
1. Convert to lowercase
2. Replace spaces with hyphens
3. Remove special characters (keep only alphanumeric and hyphens)
4. Limit to 50 characters (including ticket key prefix)
5. Remove common prefixes like "DEV-SETUP-001:", "EPIC-01-", etc.

**Examples:**
- Task: "Add ICM-42688-P gyro driver"
  - Branch: `feature/icm42688-gyro-driver`
- Task: "Fix altitude hold PID overshoot"
  - Branch: `fix/altitude-hold-pid-overshoot`

**Branch Creation Process:**

```bash
# 1. Check if branch already exists
git branch --list "[BRANCH_PREFIX]/[BRANCH_NAME]"

# If exists: Checkout existing branch
git checkout [BRANCH_PREFIX]/[BRANCH_NAME]

# If not exists:
# 2. Checkout base branch (from AGENTS.md)
git checkout [DEFAULT_GIT_BRANCH]

# 3. Pull latest changes
git pull origin [DEFAULT_GIT_BRANCH]

# 4. Create and checkout new branch
git checkout -b [BRANCH_PREFIX]/[BRANCH_NAME]

# 5. Verify branch was created
git branch --show-current
```

**If branch creation fails:**
- If base branch doesn't exist, try `develop` first, then `main` as fallback
- If both fail, ask user which branch to use as base
- Ask if they want to proceed without creating a branch

**When in a worktree:**
- Pull from git with full permission
- Handle worktree-specific branch operations

---

### 2. Commit Changes

**Conventional Commit Format:**

```
<type>: <description> (<TICKET-ID>)

- Brief bullet point of key changes
- Another key change
- Reference any fixes or important notes
```

**Commit Types:**

| Type | Description |
|------|-------------|
| `feat` | New features |
| `fix` | Bug fixes |
| `docs` | Documentation changes |
| `refactor` | Code refactoring |
| `perf` | Performance improvements |
| `test` | Test additions/modifications |
| `chore` | Build/tooling changes |

**Commit Process:**

```bash
# 1. Stage all changes
git add -A

# 2. Check status
git status

# 3. Create commit
git commit -m "feat: description of changes ($001)

- Key change 1
- Key change 2
- Key change 3"
```

**Do not add AI co-author trailers** (no `Co-authored-by: Claude/Codex/Cursor/...`). This is a
project-wide rule (`AGENTS.md` → "Project rules").

---

### 3. Push to Remote

**Push Process:**

```bash
# 1. Get current branch name
git branch --show-current

# 2. Push to remote
# If branch has upstream:
git push

# If no upstream (new branch):
git push --set-upstream origin <branch-name>
```

**Display push confirmation after successful push.**

---

### 4. Create Pull Request

PR creation is owned by the **`gh-cli`** skill. Do not re-implement template discovery, body
construction, or `gh pr create` invocation here.

**Workflow:**
1. Verify prerequisites:
   - `gh --version` (CLI installed)
   - `gh auth status` (authenticated; run with sandbox disabled if it fails)
   - `git status --short` (no uncommitted changes)
2. Generate the PR body using the **`pr-description`** skill (outputs `pr-description.md`).
3. Invoke the **`gh-cli`** skill to open the PR with `gh pr create --base [DEFAULT_GIT_BRANCH]
   --title "<type>: <summary> ([TICKET-ID])" --body-file <pr-description.md>`.

**If GitHub CLI not available or not authenticated:**
- Skip PR creation
- Display: "GitHub CLI not available or not authenticated - PR creation skipped."
- Do NOT fabricate manual links

---

## Error Handling

**If base branch doesn't exist:**
- Try `develop` first, then `main` as fallback
- If both fail, ask user which branch to use

**If push fails:**
- Check if remote is configured
- Check for authentication issues
- Display error and ask user to resolve

**If PR creation fails:**
- Defer to `gh-cli` error handling
- Suggest creating PR manually via web interface as last resort
- Do not block workflow completion

---

## Cleanup Operations

**After implementation is complete:**

```bash
# Remove any ad-hoc test scripts or temporary files
# Verify no temporary files remain before final commit
git status
```
