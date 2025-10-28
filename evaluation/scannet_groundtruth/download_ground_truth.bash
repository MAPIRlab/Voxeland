#!/usr/bin/env bash
set -e

# Minimal usage check
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <scene_suffix>   e.g., $0 0000_00"
  exit 1
fi

SUFFIX="$1"                    # e.g., "0000_00"
# Optional sanity check for the expected format ####_## (4 digits, underscore, 2 digits)
if [[ ! "$SUFFIX" =~ ^[0-9]{4}_[0-9]{2}$ ]]; then
  echo "Error: scene_suffix must match the pattern ####_## (e.g., 0000_00)"
  exit 1
fi

SCENE="scene${SUFFIX}"         # "scene0000_00"
DEST="./${SCENE}"
BASE="https://kaldir.vc.in.tum.de/scannet/v1/scans/${SCENE}"

# Create output directory if it does not exist
mkdir -p "$DEST"
cd "$DEST"

# Helper to download a file only if it does NOT already exist
download_file () {
  local fname="$1"
  if [ -f "$fname" ]; then
    echo "[Info] '$fname' already exists. Skipping download."
    return 0
  fi
  echo "[Info] Downloading '$fname'..."
  # -O writes to the exact filename
  wget -O "$fname" "${BASE}/${fname}"
}

# 1) 3D mesh for annotation (vh_clean_2)
download_file "${SCENE}_vh_clean_2.ply"

# 2) Oversegmentation for that mesh
download_file "${SCENE}_vh_clean_2.0.010000.segs.json"

# 3) Aggregated instances + semantic labels (for the annotation mesh)
download_file "${SCENE}.aggregation.json"

echo "[Done] Files available under: $(pwd)"
