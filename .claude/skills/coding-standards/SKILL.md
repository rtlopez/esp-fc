---
name: coding-standards
description: Single entrypoint for coding standards and anti-patterns. Points to the canonical standards pack under .claude/context/.
---

# Coding Standards (Y*/Z*)

Canonical coding standards and anti-patterns live under:

- Entrypoint: `.claude/context/coding-standards.md`

## How to Use in the Workflow

- Start by loading `.claude/context/coding-standards.md`, then load:
  - General full catalog: `.claude/context/coding-standards/general/index-catalog.xml` (Y*/Z*)
  - Language full catalog (as needed): `.claude/context/coding-standards/<language>/index-catalog.xml`
- In `/spec`: cite relevant **Y###-NAME** guidelines and **Z###-NAME** anti-patterns in component/API/model sections.
- In `/plan`: each P### step cites the Y/Z IDs it must follow/avoid, and includes concrete validation commands.
- In `/implement`: review cited Y/Z IDs before coding, and validate after changes (tests, lint, typecheck as applicable).
