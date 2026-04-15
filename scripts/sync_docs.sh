#!/usr/bin/env bash
# Sync REGBOT-related Obsidian notes from Mads's vault into docs/.
#
# Run this BEFORE committing in the team repo whenever you've edited the
# REGBOT Balance Assignment note or Lesson 10 in Obsidian. Teammates (who
# don't have the vault) just read docs/ directly.
#
# If the vault isn't present (teammate's machine), the script exits cleanly
# without touching docs/.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VAULT="$REPO_ROOT/../../../Obsidian/Courses/34722 Linear Control Design 1"

if [[ ! -d "$VAULT" ]]; then
    echo "[sync_docs] Obsidian vault not found at:"
    echo "            $VAULT"
    echo "[sync_docs] Nothing to sync — teammate machine, exiting."
    exit 0
fi

DOCS="$REPO_ROOT/docs"
IMG_SRC="$VAULT/Exercises/Work/regbot/Images"
IMG_DST="$DOCS/images"

mkdir -p "$DOCS" "$IMG_DST"

echo "[sync_docs] Vault: $VAULT"
echo "[sync_docs] Target: $DOCS"

# --- Markdown notes ---
# List of (source relative to vault, destination relative to docs/).
copy_md() {
    local src="$VAULT/$1"
    local dst="$DOCS/$2"
    if [[ ! -f "$src" ]]; then
        echo "  [skip] $1  (not in vault)"
        return
    fi
    cp "$src" "$dst"
    echo "  [copy] $1"
}

copy_md "Exercises/Work/regbot/REGBOT Balance Assignment.md" "REGBOT Balance Assignment.md"
copy_md "Exercises/Work/regbot/PLAN.md"                      "PLAN.md"
copy_md "Lecture Notes/Lesson 10 - Unstable Systems and REGBOT Balance.md" \
        "Lesson 10 - Unstable Systems and REGBOT Balance.md"

# --- Images ---
# Copy everything in the regbot Images folder. Use a wildcard so new plots
# get picked up automatically.
shopt -s nullglob
count=0
for f in "$IMG_SRC"/*.png; do
    cp "$f" "$IMG_DST/"
    count=$((count + 1))
done
shopt -u nullglob
echo "  [copy] $count image(s) -> docs/images/"

echo "[sync_docs] Done."
