# Worktree and Environment Setup

Setup guide for git worktrees and PlatformIO build environments before implementing tasks in this
repo (C++17 embedded firmware, PlatformIO + Arduino framework).

---

## Detecting a Git Worktree

Before starting any implementation, detect if running in a worktree:

```bash
# .git is a FILE in worktrees, DIRECTORY in main repos
[ -f .git ] && echo "WORKTREE" || echo "MAIN_REPO"
```

Worktrees are safe to use — each has its own working tree. PlatformIO build cache (`.pio/`) is
per-directory so worktrees do not share build artifacts.

---

## Build Environment Setup

This repo uses **PlatformIO** (`pio`). No Python virtualenv or npm install required for firmware
work. PlatformIO manages the toolchain automatically.

### 1. Verify PlatformIO is installed

```bash
pio --version
```

If missing, install via:

```bash
pip install platformio
# or via the PlatformIO IDE extension
```

### 2. Verify build works

```bash
# Recommended target (ESP32-S3 — Project PHI hardware)
pio run -e esp32s3

# All targets (CI equivalent — 7 platforms)
pio run
```

### 3. Verify native tests compile and run

```bash
pio test -e native
```

Native tests require no hardware — they compile to x86 and run locally.

---

## Environment Verification Checklist

Before proceeding with implementation:

- [ ] `pio --version` works
- [ ] `pio run -e esp32s3` compiles cleanly (no errors, warnings may exist)
- [ ] `pio test -e native` passes
- [ ] No uncommitted changes in `lib/EscDriver/` or `lib/AHRS/` that would affect the build

---

## Running Build & Flash Commands

```bash
# Build specific target
pio run -e esp32s3          # ESP32-S3 (Project PHI)
pio run -e esp32            # ESP32 classic
pio run -e native           # native (tests only)

# Flash to hardware
pio run -e esp32s3 -t upload

# Run native unit tests
pio test -e native

# Build all CI targets
pio run
```

---

## Key Paths

| Path | Purpose |
|------|---------|
| `src/main.cpp` | Arduino entry point |
| `lib/Espfc/src/` | Main firmware library |
| `lib/EscDriver/` | ESC protocol library |
| `lib/AHRS/` | Attitude estimators |
| `test/` | Unity unit tests (native) |
| `platformio.ini` | All build environments & flags |

---

## Environment Error Handling

| Error | Action |
|-------|--------|
| `pio` not found | Install PlatformIO: `pip install platformio` |
| Toolchain download fails | Check internet connection; retry `pio run` |
| `native` test build fails | Check `#ifndef UNIT_TEST` guards around hardware headers |
| ESP32-S3 build fails | Check `platformio.ini` for correct env flags |
| Flash fails | Check USB connection, verify correct COM/tty port |
