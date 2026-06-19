# Validation Reference

Patterns for validating acceptance criteria during task implementation.

---

## Validation Process

**For each acceptance criterion:**

1. **Run validation command.** Use the appropriate `pio` command for the scope being tested.

   ```bash
   # Unit tests (native, no hardware required)
   pio test -e native

   # Build verification for target platform
   pio run -e esp32s3

   # Build all CI targets
   pio run
   ```

2. **Display result with status:**
   - `✓ Acceptance Criterion: [Description]`
   - `  Status: [X] out of [X] tests passed` (for native tests)
   - `  Status: BUILD SUCCESS` (for build verification)

3. **If validation passes:**
   - Mark checkbox ✅ automatically
   - Continue to next criterion (NO user interaction)

4. **If validation fails:**
   - Display: `✗ Acceptance Criterion: [Description]`
   - Display error output
   - **STOP immediately**
   - **Triage the problem**
   - **Attempt to fix** if within capabilities
   - **Escalate to user** if beyond capabilities

---

## Testing Requirements

### Native Unit Tests

- Tests live in `test/` and use the **Unity** framework.
- Test files compile to x86 — no hardware required.
- Every hardware-specific header must be guarded with `#ifndef UNIT_TEST`.
- Run with: `pio test -e native`

```bash
# Run all native tests
pio test -e native

# Check a specific test file compiles
pio test -e native -f test_filters
```

### Build Verification

New sensor drivers and code changes must compile cleanly for the primary target:

```bash
# Primary target
pio run -e esp32s3

# Full CI suite (all 7 platforms)
pio run
```

### Hot-Path Validation

For changes touching the gyro/PID path, also verify:

```bash
# After flashing, check stats via CLI
# Connect at 115200 baud, run:
#   stats
# Look for: CPU load < 70%, no loop overruns
```

---

## CLI Verification (Serial)

For runtime validation after flashing:

```bash
# Connect to board at 115200 baud
# Run in CLI:
stats          # CPU load, loop rate
dump           # Full config dump
version        # Firmware version
```

---

## Troubleshooting Workflow

When validation fails:

1. **Document issue** — Capture exact error output
2. **Analyze causes** — Common causes in `.claude/rules/05-session-loop.md`
3. **Create plan** — For each cause: verify, solution
4. **Present to user** — Ask before proceeding
5. **Execute with approval** — Work through systematically
6. **Verify fix** — Re-run failing command

---

## Common Failure Patterns

| Failure | Likely Cause | Fix |
|---------|-------------|-----|
| `UNIT_TEST` build fails | Hardware header outside `#ifndef UNIT_TEST` | Wrap with guard |
| ESP32-S3 build fails | Missing `FAST_CODE_ATTR` | Add annotation to hot-path function |
| Linker error on IRAM | Virtual call in fast path | Replace with direct/template dispatch |
| Test segfault | Uninitialized mock or missing stub | Add minimal stub in `test/` |

---

## Status Indicators

| Status | Indicator | Meaning |
|--------|-----------|---------|
| Complete | ✅ | All criteria pass |
| In Progress | 🔄 | Currently working |
| Not Started | ⬜ | Waiting or not begun |
| Blocked | ⚠️ | Cannot proceed |
