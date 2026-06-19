# .claude/context/AGENTS.md
## Objective
Maintain a machine-navigable context directory. Optimise for:
- unambiguous structure
- stable IDs and references
- low token overhead
- deterministic formatting (easy diffs)
Do not change semantic meaning unless explicitly asked.

## File conventions (XML)
- All context files MUST be valid XML 1.0, UTF-8, LF line endings.
- Indentation: 2 spaces, no tabs.
- Deterministic ordering:
  - Attributes in a stable order: id, title, kind, version, updated, status, path (when relevant).
  - Entries/rules sorted by id ascending unless the user requested a specific ordering.

## Required metadata
- Every file has exactly one top-level element with:
  - id (stable, never rename once published)
  - title (short)
  - kind (manifest|ruleset|glossary|playbook|other)
  - version (integer, bump when content changes)
  - updated (YYYY-MM-DD, update when content changes)
  - status (active|deprecated)

## References and navigation
- Never encode references as comma-separated text.
- Use explicit references:
  - <ref id="..."/> for rule-to-rule links
  - <doc_ref path="..."/> for file-to-file links (only in manifests)
- Ensure referential integrity:
  - Every doc_ref path exists
  - Every ref id resolves to exactly one rule in the repository

## Rule schema (avoid generic "field name=…")
- Do NOT use <field name="..."> patterns.
- Use typed elements for predictable parsing:
  - <desc>, <why_bad>, <when_to_use>, <exceptions>
  - <better><ref/></better>
  - <keywords><kw/></keywords>
  - <examples><example kind="bad|good"><code lang="..."><![CDATA[...]]></code></example></examples>

## Code examples
- Always store code examples in:
  <code lang="python|sql|bash|..."><![CDATA[...]]></code>
- Preserve indentation and trailing newlines within CDATA.
- Never use backticks inside XML to denote code.

## Manifest expectations (index)
- Manifest MUST provide cheap routing without opening other files:
  - <keywords>, <contains>, optional <token_estimate>
- Each manifest entry includes:
  - id, title, kind, path, short purpose, keywords
- Keep descriptions short (1–2 lines).

## Validation and checks (mandatory)
After any modification:
1) run XML validation (schema validation if available)
2) run referential integrity checks (paths + ref IDs)
3) ensure no duplicate IDs
If the repo lacks these checks, add:
- .claude/tools/context/validate_context.py
- a Taskfile target: `task context:validate`
- CI hook if present
If formatting is off (canonical indentation / inline single-line tags), run:
- `task context:format`

## Output expectations
- Provide a short change summary.
- List any breaking changes explicitly.
- If anything is ambiguous, make the smallest safe change and document it.
