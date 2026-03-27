#!/usr/bin/env bash

echo "=== Start updating Modules ==="

for dir in Modules/*/; do
    [ -d "$dir" ] || continue

    echo "🔹 Processing module: $dir"

    cd "$dir" || continue

    if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        echo "  Switching to 'main' branch..."
        git switch main

        echo "  Pulling latest changes..."
        git pull
    else
        echo "  ⚠ Not a git repository, skipping."
    fi

    cd - >/dev/null || exit
done

echo "=== All modules updated ==="
