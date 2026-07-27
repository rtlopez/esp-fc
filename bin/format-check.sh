#!/usr/bin/env bash

VERBOSE=0
if [ "$1" = "-v" ]; then
    VERBOSE=1
    shift
fi

ROOT_DIR="${1:-.}"

if command -v clang-format-20 >/dev/null 2>&1; then
    CLANG_FORMAT="clang-format-20"
elif command -v clang-format >/dev/null 2>&1; then
    CLANG_FORMAT="clang-format"
else
    echo "Error: neither clang-format-20 nor clang-format was found in PATH" >&2
    exit 1
fi

find "$ROOT_DIR" \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -print0 |
while IFS= read -r -d '' file; do
    if [ "$VERBOSE" -eq 1 ]; then
        output=$("$CLANG_FORMAT" --dry-run --Werror "$file" 2>&1)
    else
        "$CLANG_FORMAT" --dry-run --Werror "$file" >/dev/null 2>&1
    fi
    if [ $? -ne 0 ]; then
        echo "Needs formatting: $file"
        if [ "$VERBOSE" -eq 1 ]; then
            echo "$output"
        fi
    fi
done
