#!/usr/bin/env bash
# Run unit tests with coverage and generate reports.
# Usage: bin/coverage.sh [--html]
#   coverage.xml, test-report.xml - SonarQube reports
#   coverage/index.html           - HTML report (with --html)

set -euo pipefail

cd "$(dirname "$0")/.."

HTML=0
[[ "${1:-}" = "--html" ]] && HTML=1

if command -v gcovr >/dev/null 2>&1; then
    GCOVR=gcovr
elif [[ -x /tmp/gv/bin/gcovr ]]; then
    GCOVR=/tmp/gv/bin/gcovr
else
    echo "Error: gcovr not found, install it with: sudo apt install gcovr (or: pipx install gcovr)" >&2
    exit 1
fi

BUILD_DIR=.pio/build/coverage
GCOVR_ARGS=(--root . --filter 'lib/' --exclude 'lib/CmsisDsp/' --exclude 'lib/betaflight/')

find "$BUILD_DIR" -name '*.gcda' -delete 2>/dev/null || true

pio test -e coverage --junit-output-path .pio/junit.xml
python3 bin/junit_to_sonar.py .pio/junit.xml test-report.xml coverage
"$GCOVR" "${GCOVR_ARGS[@]}" --sonarqube coverage.xml "$BUILD_DIR"

if [[ "$HTML" -eq 1 ]]; then
    mkdir -p coverage
    "$GCOVR" "${GCOVR_ARGS[@]}" --html-details coverage/index.html "$BUILD_DIR"
    echo "HTML report: coverage/index.html"
fi

"$GCOVR" "${GCOVR_ARGS[@]}" --txt --print-summary "$BUILD_DIR" | tail -n 5
