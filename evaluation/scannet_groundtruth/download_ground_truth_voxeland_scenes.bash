#!/usr/bin/env bash
set -euo pipefail

# Runs download_ground_truth.bash for every directory named scene####_## inside ../voxeland_output

# Resolve paths relative to this script
THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DL_SCRIPT="$THIS_DIR/download_ground_truth.bash"
VOX_DIR="$THIS_DIR/../voxeland_output"

# Basic checks
if [[ ! -x "$DL_SCRIPT" ]]; then
  echo "Error: download_ground_truth.bash not found or not executable at: $DL_SCRIPT"
  exit 1
fi
if [[ ! -d "$VOX_DIR" ]]; then
  echo "Error: voxeland_output directory not found at: $VOX_DIR"
  exit 1
fi

# Make the glob return empty rather than the literal pattern when no match
shopt -s nullglob

found_any=false
for dirpath in "$VOX_DIR"/scene[0-9][0-9][0-9][0-9]_[0-9][0-9]; do
  [[ -d "$dirpath" ]] || continue
  found_any=true
  scenedir="$(basename "$dirpath")"      # e.g., scene0000_01
  suffix="${scenedir#scene}"             # -> 0000_01
  echo "[Info] Processing $scenedir (suffix: $suffix)"
  "$DL_SCRIPT" "$suffix"
done

if ! $found_any; then
  echo "No matching directories found under $VOX_DIR (expected names like 'scene0000_01')."
fi
