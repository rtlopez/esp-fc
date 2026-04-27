#!/usr/bin/env bash

ROOT_DIR="${1:-.}"

find "$ROOT_DIR" \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -print0 |
while IFS= read -r -d '' file; do
    clang-format --dry-run --Werror "$file" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Needs formatting: $file"
    fi
done
