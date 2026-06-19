---
title: Coding Standards Overview
---

# Coding Standards (XML Pack)

**Purpose:** agent-first coding standards with:
- stable IDs (Y*/Z*/PY*/PZ*/JY*/JZ*/TY*/TZ*)
- XML sources for rules + metadata
- per-language `index-catalog.xml` for one-shot discovery

## Entry Points

- Root entrypoint: `.claude/context/coding-standards.md`
- Per-language:
  - `./<language>/overview.md` (language entrypoint)
  - `./<language>/index-catalog.xml` (all rule IDs + 1-line desc; generated)
  - `./<language>/index.xml` (hundreds metadata; source of truth)
  - `./<language>/*/index.xml` (tens metadata; source of truth)

## Regenerate Catalogs

```bash
python .claude/tools/context/generate_catalog.py
python .claude/tools/context/generate_catalog.py --check
```
