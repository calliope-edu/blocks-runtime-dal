#!/bin/bash
set -e
cd /src
git config --global --add safe.directory /src 2>/dev/null || true
echo "=== target (from copied .yotta.json) ==="
cat .yotta.json | grep target
echo "=== yotta build (OFFLINE: cached yotta_modules + yotta_targets, no 'yotta up') ==="
yotta build 2>&1 | tail -50
echo "=== artifacts ==="
ls -la build/*/source/*.hex 2>/dev/null
