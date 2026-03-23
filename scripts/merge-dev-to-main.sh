#!/bin/bash
# Merge dev into main, automatically excluding logs/
#
# Usage: ./scripts/merge-dev-to-main.sh

set -euo pipefail

CURRENT_BRANCH=$(git branch --show-current)
if [ "$CURRENT_BRANCH" != "main" ]; then
    echo "Error: Must be on main branch. Currently on '$CURRENT_BRANCH'"
    exit 1
fi

echo "Merging dev into main (excluding logs/)..."

# Merge without auto-commit
git merge dev --no-commit --no-ff 2>/dev/null || true

# Remove logs if they came through from dev
if git diff --cached --name-only | grep -q '^logs/'; then
    git reset HEAD -- logs/ >/dev/null 2>&1 || true
    rm -rf logs/
    echo "  -> logs/ excluded from merge"
fi

# Check if there are changes to commit
if git diff --cached --quiet 2>/dev/null; then
    echo "No changes to merge (main is up to date with dev)"
    git merge --abort 2>/dev/null || true
    exit 0
fi

git commit -m "Merge dev into main (logs excluded)"
echo "Done! Merge complete."
