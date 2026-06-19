# Context Directory

This directory contains canonical reference material intended for consumption by AI
agents (and humans) working in this repository.

It is the single source of truth for coding standards and anti-patterns. Commands and
skills should reference these documents rather than duplicating their content elsewhere.

## Structure

- `README.md` describes the layout of `.claude/context/`.
- `AGENTS.md` is the authoring guide for the XML files in this directory. It is distinct from the repo-root `AGENTS.md`.
- `index.xml` is the top-level manifest; every entry resolves to one of the files listed below.
- `schema/context.xsd` is the XSD the XML files validate against.
- `coding-standards.md` is the human entrypoint for the coding standards packs.
- `coding-standards/` contains the canonical standards/anti-pattern catalogs (XML, plus `overview.md`).

## Entry Points

- Coding standards: `.claude/context/coding-standards.md`

## Structure Principles (Agent-First)

- **Small entrypoints:** provide a minimal index before any large content.
- **Selective loading:** index entries include 1-line summaries to guide what to load.
- **Shallow defaults:** only load detailed files when needed.
- **Stable links:** prefer relative paths and keep filenames consistent.

## File Size and Hierarchy Rules

- **Target size:** keep context files at **80 lines or fewer**.
- **Split triggers:** split files that exceed 80 lines into a folder with:
  - `index.md` (topic overview + links with summaries)
  - subtopic or per-rule files
- **Exception:** coding standards under `.claude/context/coding-standards/**` are XML-only
  (except `overview.md`); see `.claude/context/coding-standards/overview.md`.
- **Hierarchy:** root index → domain index → topic index → subtopic/rule files.
- **Naming:** use consistent ID prefixes and wildcard folders (e.g., `y1xx`, `py2xx`)
  to signal ranges and reduce scanning time.

## Usage Convention

When writing `spec.md` and `plan.md`, cite the relevant IDs by language:

- General: Y### guidelines and Z### anti-patterns
- Python: PY### guidelines and PZ### anti-patterns
