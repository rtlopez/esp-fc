# Agent Skills Index

This index provides metadata for all available skills. Skills are loaded on-demand to optimize context use.

## Invocation Model

All workflows support two equivalent invocation styles:

- Slash-command tools: `/name`
- Skill-invocation tools: `$name`

Behavior is canonical in `.claude/skills/<name>/SKILL.md`. Command files in `.claude/commands/`
are thin entrypoints that forward to same-name skills.

## Workflow Skills

### spec

**Purpose**: Create feature-level implementation specs (`spec.md`) with requirements, component design, and test strategy
**Invoke as**: `/spec` or `$spec`
**Skill file**: `.claude/skills/spec/SKILL.md`

### plan

**Purpose**: Convert `spec.md` into executable `plan.md` with ordered steps and validation
**Invoke as**: `/plan` or `$plan`
**Skill file**: `.claude/skills/plan/SKILL.md`

### implement

**Purpose**: Execute plan steps with validation and progress tracking
**Invoke as**: `/implement` or `$implement`
**Skill file**: `.claude/skills/implement/SKILL.md`

### adr

**Purpose**: Create Architecture Decision Records under `docs/adr/`
**Invoke as**: `/adr` or `$adr`
**Skill file**: `.claude/skills/adr/SKILL.md`

### pr-description

**Purpose**: Generate a PR description (`pr-description.md`) from branch commits and diff
**Invoke as**: `/pr-description` or `$pr-description`
**Skill file**: `.claude/skills/pr-description/SKILL.md`

### pr-review

**Purpose**: Review pull requests against intent, spec/plan traceability, and standards
**Invoke as**: `/pr-review` or `$pr-review`
**Skill file**: `.claude/skills/pr-review/SKILL.md`

## Supporting Skills

### coding-standards

**Purpose**: Entry point to general coding standards and anti-patterns catalogs (Y*/Z*)
**Skill file**: `.claude/skills/coding-standards/SKILL.md`

### managing-git

**Purpose**: Branch/commit/push/PR operations for workflow automation
**Skill file**: `.claude/skills/managing-git/SKILL.md`

### gh-cli

**Purpose**: Safe and reliable GitHub CLI patterns
**Skill file**: `.claude/skills/gh-cli/SKILL.md`
